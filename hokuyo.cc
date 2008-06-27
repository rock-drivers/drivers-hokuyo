#include "hokuyo.hh"

#include <map>
#include <string>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <sstream>

#include <termio.h>
#include <sys/types.h>
#include <sys/time.h>
#include <time.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>

using namespace std;

static string printable_com(string const& buffer)
{
    char const* str = buffer.c_str();
    size_t str_size = buffer.size();
    ostringstream result;
    for (size_t i = 0; i < str_size; ++i)
    {
        if (str[i] == '\n')
            result << "\\n";
        else if (str[i] == '\r')
            result << "\\r";
        else
            result << str[i];
    }
    return result.str();
}

struct URG::StatusCode
{
    char const* urg_status;
    URG::ERROR_CODES ret_code;
};

struct ReturnValueDescription
{
    URG::ERROR_CODES ret_code;
    char const* description;
};

static ReturnValueDescription URG_RETURN_DESCRIPTION[] = {
    { URG::OK,                         "OK" },

    { URG::INTERNAL_COM_FAILED,        "INTERNAL_COM_FAILED" },
    { URG::DEVICE_BUFFER_OVERFLOW,     "DEVICE_BUFFER_OVERFLOW" },
    { URG::BAD_COMMAND,                "BAD_COMMAND" },
    { URG::STRING_TOO_LONG,            "STRING_TOO_LONG" },
    { URG::BAD_STRING,                 "BAD_STRING" },
    { URG::FIRMWARE_UPDATE,            "FIRMWARE_UPDATE" },
    { URG::LASER_MALFUNCTION,          "LASER_MALFUNCTION" },
    { URG::HARDWARE_FAILURE,           "HARDWARE_FAILURE" },

    { URG::BAD_REPLY,                  "BAD_REPLY" },
    { URG::DRIVER_BUFFER_OVERFLOW,     "DRIVER_BUFFER_OVERFLOW" },
    { URG::PROVIDED_BUFFER_TOO_SMALL,  "PROVIDED_BUFFER_TOO_SMALL" },
    { URG::BAD_STATE,                  "BAD_STATE" },
    { URG::READ_FAILED,                "READ_FAILED" },
    { URG::READ_TIMEOUT,               "READ_TIMEOUT" },
    { URG::WRITE_FAILED,               "WRITE_FAILED" },
    { URG::WRITE_TIMEOUT,              "WRITE_TIMEOUT" },
    { URG::BAD_RATE,                   "BAD_RATE" },
    { URG::BAD_HOST_RATE,              "rate not supported on this host" },
    { URG::NOT_SCIP2_CAPABLE,          "this device is not SCIP2. Upgrade firmware" },
    { URG::INCONSISTEN_RANGE_COUNT,    "did not get the expected count of ranges" },

    { URG::UNKNOWN,                    "UNKNOWN" },

    { URG::END, 0 }
};

static URG::StatusCode URG_COMMON_OK[] = {
    { "00", URG::OK },
    { "99", URG::OK },
    { 0, URG::END }
};

static URG::StatusCode URG_COMMON_STATUS_CODES[] = {
    { "00", URG::OK },
    { "99", URG::OK },
    { "0I", URG::FIRMWARE_UPDATE },
    { "0A", URG::INTERNAL_COM_FAILED },
    { "0B", URG::DRIVER_BUFFER_OVERFLOW },
    { "0C", URG::BAD_COMMAND },
    { "0D", URG::BAD_COMMAND },
    { "0E", URG::BAD_COMMAND },
    { "0F", URG::BAD_COMMAND },
    { "0G", URG::STRING_TOO_LONG },
    { "0H", URG::BAD_STRING },
    { 0, URG::END }
};

/* WARNING: the following error codes are one-byte status -- even though SLIP
 * specification tells all codes are two-bytes */
static URG::StatusCode URG_STATUS_CODES_SCIP1[] = {
    { "0", URG::OK },
    { "A", URG::INTERNAL_COM_FAILED },
    { "B", URG::DRIVER_BUFFER_OVERFLOW },
    { "C", URG::BAD_COMMAND },
    { "D", URG::BAD_COMMAND },
    { "E", URG::BAD_COMMAND },
    { "F", URG::BAD_COMMAND },
    { "G", URG::STRING_TOO_LONG },
    { "H", URG::BAD_STRING },
    { 0, URG::END }
};

static URG::StatusCode URG_SCIP2_STATUS_CODES[] = {
    { "0E", URG::OK }, // 0E means "already in SCIP2"
    { 0, URG::END }
};

static URG::StatusCode URG_BM_STATUS_CODES[] = {
    { "01", URG::LASER_MALFUNCTION },
    { "02", URG::OK },
    { 0, URG::END }
};
static URG::StatusCode URG_SS_STATUS_CODES[] = {
    { "01", URG::BAD_RATE },
    { "02", URG::BAD_RATE },
    { "03", URG::OK },
    { 0, URG::END }
};

/** Description of a command without any parameter */
struct URG::SimpleCommand
{
    /** The command two-character name as sent on the wire */
    char const* name;
    /** An array of specific return codes, or NULL if there is none */
    StatusCode* specific_codes;
};

static URG::SimpleCommand URG_BM =  { "BM", URG_BM_STATUS_CODES };
static URG::SimpleCommand URG_QUIT =  { "QT", 0 };
static URG::SimpleCommand URG_SCIP2 = { "SCIP2.0", URG_SCIP2_STATUS_CODES };
static URG::SimpleCommand URG_RESET = { "RS", 0 };

char const* URG::errorString(int error_code)
{
    for (ReturnValueDescription const* code = URG_RETURN_DESCRIPTION; code->description; ++code)
    {
        if (error_code == code->ret_code)
            return code->description;
    }
    return "no description for that code";
}

URG::URG()
    : m_error(OK)
    , fd(-1)
    , baudrate(19200)
    , internal_buffer_size(0)
{
    m_last_status[0] = 
        m_last_status[1] = 
        m_last_status[2] = 0;
}


// Parses an int of x bytes in the hokyo format
static unsigned int parseInt(int bytes, char const*& s){
    unsigned int ret=0;
    int j = bytes-1;
    int i;
    for (i=0; i<bytes;){
        if (*s==0||*s=='\n'){
            s = 0;
            return 0;
        }
        if (*(s+1)=='\n'){ //check for a wrapped line
            s++;
        } else {
            unsigned int c = *s-0x30;
            ret += c<<(6*j);
            i++;
            j--;
        }
        s++;
    }
    return ret;
}

bool URG::write(char const* string, int timeout)
{
    char buffer[MAX_PACKET_SIZE];
    sprintf(buffer, "\n%s\n", string);
    size_t cmd_size = strlen(buffer);

    size_t written_size = 0;
    while (written_size != cmd_size)
    {
        fd_set set;
        FD_ZERO(&set);
        FD_SET(fd, &set);

        timeval timeout_spec = { timeout / 1000, (timeout % 1000) * 1000 };
        int ret = select(fd + 1, NULL, &set, NULL, &timeout_spec);
        if (ret < 0)
            return error(WRITE_FAILED);
        else if (ret == 0)
            return error(WRITE_TIMEOUT);

        int single_write = ::write(fd, buffer + written_size, cmd_size - written_size);
        if (single_write < 0)
            return error(WRITE_FAILED);

        written_size += single_write;
    }

    return true;
}

bool URG::infoCommand(map<string, string>& result, char const* cmd, bool scip1)
{
    char buffer[MAX_PACKET_SIZE];
    size_t cmd_size = strlen(cmd);
    if (!write(cmd))
        return false;

    int packet_size = URG::readAnswer(buffer, MAX_PACKET_SIZE, cmd);
    if (packet_size < 6)
        return false;
    if (!parseErrorCode(buffer + cmd_size + 1, 0))
        return false;

    // Find the \n after buffer + cmd_size + 1
    char const* field_start = buffer + cmd_size + 1;
    for (; *field_start != '\n'; ++field_start);
    
    for (char const* end = field_start + 1; *end != '\n' && (end - buffer) < packet_size; )
    {
        char const* start_point = end;
        char const* name_end = 0;
        char const* value_end = 0;
        char const* field_end = 0;

        // Search for the next "\n" mark
        for (; !field_end && (end - buffer) != packet_size; ++end)
        {
            if (!name_end && *end == ':')
                name_end = end;
            else if (!value_end && *end == ';')
                value_end = end;
            else if (!field_end && *end == '\n')
                field_end = end;
        }
        // here, +end+ is field_end + 1

        if (end - buffer == packet_size)
            return error(BAD_REPLY);
        if (!name_end || !field_end)
            return error(BAD_REPLY);

        if (!value_end)
        {
            if (!scip1)
                return error(BAD_REPLY);
            value_end = field_end;
        }

        string name(start_point, name_end);
        string value(name_end + 1, value_end);

        result.insert( make_pair( name, value ) );
    }

    return true;
}

bool URG::error(ERROR_CODES error_code)
{
    m_error = error_code;
    if (m_error == URG::OK)
        return true;
    else
        return false;
}
bool URG::readInfo()
{
    map<string, string> fields;
    if ( !URG::infoCommand(fields, "II") )
        return false;
    if ( !URG::infoCommand(fields, "VV") )
        return false;
    if ( !URG::infoCommand(fields, "PP") )
        return false;

    if (fields["STAT"] != "Sensor works well.")
        return error(BAD_STATE);

    m_info.values     = fields;
    m_info.dMin       = atoi(fields["DMIN"].c_str());
    m_info.dMax       = atoi(fields["DMAX"].c_str());
    m_info.resolution = atoi(fields["ARES"].c_str());
    m_info.stepMin    = atoi(fields["AMIN"].c_str());
    m_info.stepMax    = atoi(fields["AMAX"].c_str());
    m_info.stepFront  = atoi(fields["AFRT"].c_str());
    m_info.motorSpeed = atoi(fields["SCAN"].c_str());
    return true;
}

int URG::readAnswer(char* buffer, size_t buffer_size, char const* expected_cmd, int timeout)
{
    char const* cmds[] = { expected_cmd, 0 };
    return readAnswer(buffer, buffer_size, cmds, timeout);
}

int URG::readAnswer(char* buffer, size_t buffer_size, char const** expected_cmds, int timeout)
{
    timeval start_time;
    gettimeofday(&start_time, 0);
    while(true) {
        timeval current_time;
        gettimeofday(&current_time, 0);

        int elapsed = 
            (current_time.tv_sec - start_time.tv_sec) * 1000
            + (static_cast<int>(current_time.tv_usec) -
                    static_cast<int>(start_time.tv_usec)) / 1000;
        if (elapsed > timeout)
        {
            error(READ_TIMEOUT);
            return -1;
        }


        int packet_size = read(buffer, buffer_size);
        if (packet_size < 0)
            return -1;
        
        for (char const** cmd = expected_cmds; *cmd; ++cmd)
        {
            if (packet_size > strlen(*cmd) && strncmp(buffer, *cmd, strlen(*cmd)) == 0)
                return packet_size;
        }

        if (packet_size)
        {
            string message;
            if (packet_size > 50)
                message = string(buffer, 50) + "...";
            else
                message = string(buffer, packet_size);

            std::cerr << "ignored packet " << printable_com(message) << endl;
            continue;
        }

        fd_set set;
        FD_ZERO(&set);
        FD_SET(fd, &set);

        timeval timeout_spec = { timeout / 1000, (timeout % 1000) * 1000 };
        int ret = select(fd + 1, &set, NULL, NULL, &timeout_spec);
        if (ret < 0)
        {
            error(READ_FAILED);
            return -1;
        }
        else if (ret == 0)
        {
            error(READ_TIMEOUT);
            return -1;
        }

    }
}


int URG::read(char* buffer, size_t buffer_size) {
    m_error = OK;
    if (buffer_size < MAX_PACKET_SIZE)
    {
        error(PROVIDED_BUFFER_TOO_SMALL);
        return -1;
    }

    // If there is none, go read some data ...
    bool was_linefeed = false;

    if (internal_buffer_size > 0)
    {
        //cerr << internal_buffer_size << " bytes remaining in internal buffer" << endl;

        // Search for a double \n in the remains of the buffer
        for (size_t i = 1; i < internal_buffer_size; ++i)
        {
            if (internal_buffer[i - 1] == '\n' && internal_buffer[i] == '\n')
            {
                memcpy(buffer, internal_buffer, i + 1);
                internal_buffer_size -= i + 1;
                memcpy(internal_buffer, internal_buffer + i + 1, internal_buffer_size);
                //cerr << "packet: "    << printable_com(string(buffer, i + 1)) << endl;
                //cerr << "remaining: " << printable_com(string(internal_buffer, internal_buffer_size)) << " (" << internal_buffer_size << ")" << endl;
                return i + 1;
            }
        }

        memcpy(buffer, internal_buffer, internal_buffer_size);
        was_linefeed = (internal_buffer[internal_buffer_size - 1] == '\n');
    }

    char* b = buffer + internal_buffer_size;
    internal_buffer_size = 0;
    while (true) {
        int c = ::read(fd, b, MAX_PACKET_SIZE - (b - buffer));
        if (c > 0) {
            //cerr << "received: " << printable_com(string(b, c)) << " (" << c << ")" << endl;
            //cerr << "buffer:   "  << printable_com(string(buffer, b + c)) << " (" << b + c - buffer << ")" << endl;
            for (size_t i = 0; i < c; i++) {
                if (was_linefeed && b[i] == '\n') {
                    ++i;
                    memcpy(internal_buffer, b + i, c - i);
                    internal_buffer_size = c - i;
                    //cerr << "packet:    " << printable_com(string(buffer, b + i)) << endl;
                    //cerr << "remaining: " << printable_com(string(internal_buffer, internal_buffer_size)) << " (" << internal_buffer_size << ")" << endl;
                    return b + i - buffer;
                }
                was_linefeed = (b[i] == '\n');
            }
            b += c;
        }
        else if (c < 0)
        {
            if (errno == EAGAIN)
            {
                internal_buffer_size = b - buffer;
                memcpy(internal_buffer, buffer, internal_buffer_size);
                return 0;
            }

            error(READ_FAILED);
            return -1;
        }

        if (b == buffer + MAX_PACKET_SIZE)
        {
            // something wrong on the line ...
            fprintf(stderr, "current packet too large (%i > %i)\n", b - buffer, MAX_PACKET_SIZE);
            error(READ_FAILED);
            return -1;
        }
    }

    // Never reached
}

bool URG::parseErrorCode(char const* code, StatusCode const* specific_codes)
{
    StatusCode const* try_codes[3] = { URG_COMMON_OK, specific_codes, URG_COMMON_STATUS_CODES };

    if (code[1] == '\n')
    {
        if (code[0] == '0')
            return true;
        else
            return error(NOT_SCIP2_CAPABLE);
    }

    m_last_status[0] = code[0];
    m_last_status[1] = code[1];

    for (int i = 0; i < 3; ++i)
    {
        StatusCode const* code_set = try_codes[i];
        if (!code_set)
            continue;

        for (; code_set->urg_status; ++code_set)
        {
            if (strncmp(code_set->urg_status, code, 2) == 0)
                return error(code_set->ret_code);
        }
    }
    cerr << "unknown error code " << printable_com(string(code, 2)) << endl;
    return error(UNKNOWN);
}

bool URG::simpleCommand(SimpleCommand const& cmd, int timeout) {
    char buf[MAX_PACKET_SIZE];
    size_t cmd_size = strlen(cmd.name);
    if (!write(cmd.name))
        return false;

    int packet_size = readAnswer(buf, MAX_PACKET_SIZE, cmd.name, timeout);
    if (packet_size < 0)
        return false;
    return parseErrorCode(buf + cmd_size + 1, cmd.specific_codes);
}

static bool set_host_baudrate(int fd, int brate) {
    struct termios termios_p;
    if(tcgetattr(fd, &termios_p)){
        perror("Failed to get terminal info \n");    
        return false;
    }

    if(cfsetispeed(&termios_p, brate)){
        perror("Failed to set terminal input speed \n");    
        return false;
    }

    if(cfsetospeed(&termios_p, brate)){
        perror("Failed to set terminal output speed \n");    
        return false;
    }

    if(tcsetattr(fd, TCSANOW, &termios_p)) {
        perror("Failed to set speed \n");    
        return false;
    }
    return true;
}

bool URG::fullReset() {
    cerr << "Resetting scanner..." << flush;
    speed_t baudrates[]={B19200, B57600, B115200};
    const int baudrates_count = 3;

    int i;
    for (i=0; i < baudrates_count; i++){
        if (!set_host_baudrate(fd, baudrates[i]))
            return error(URG::BAD_HOST_RATE);;

        m_error = OK;
        if (simpleCommand(URG_SCIP2, 10000))
        {
            if (!simpleCommand(URG_QUIT, 10000))
                return false;
            if (!simpleCommand(URG_RESET, 10000))
                return false;
            break;
        }
        else if (error() == NOT_SCIP2_CAPABLE)
        {
            map<string, string> fields;
            infoCommand(fields, "V");
            return error(NOT_SCIP2_CAPABLE);
        }
    }

    if (i == baudrates_count)
        return false;

    if (!set_host_baudrate(fd, B19200))
        return error(URG::BAD_HOST_RATE);;

    // Set baud rate to default
    cerr << " done" << endl;
    baudrate = 19200;
 
    // Have to wait after the reset, in order to get the device up and working
    timespec tv = { 1, 0 };
    nanosleep(&tv, &tv);

    return true;
}

static int tc_baudrateSetting(int brate)
{
    switch(brate) {
        case(19200):
            return B19200;
        case(57600):
            return B57600;
        case(115200):
            return B115200;
        default:
            printf("Invalid Baudrate specified \n");
            return 0;
    }
}

bool URG::setBaudrate(int brate){
    char cmd[MAX_PACKET_SIZE];
    //char answer[bufsize];
    int i;
    int status;
    int sucess = -1;

    speed_t newbrate = tc_baudrateSetting(brate);
    if (!newbrate)
        return error(BAD_RATE);


    //switch to current baudrate
    if(! set_host_baudrate(fd, tc_baudrateSetting(baudrate)))
        return error(URG::BAD_HOST_RATE);;

    sprintf(cmd, "SS%06d", brate);
    SimpleCommand cmd_obj = { cmd, URG_SS_STATUS_CODES };
    if (URG::simpleCommand(cmd_obj))
    {
        if (!set_host_baudrate(fd, newbrate))
            return error(URG::BAD_HOST_RATE);;

        baudrate = brate;
    }

    // Have to wait after the reset, in order to get the communication link reset
    timespec tv = { 1, 0 };
    nanosleep(&tv, &tv);

    if (!URG::readInfo())
        return error(BAD_STATE);
    return baudrate == brate;
}

static bool setNonBlocking(int fd) {
    //switch back to nonblocking
    if(fcntl(fd, F_SETFL, O_RDWR | O_NONBLOCK | O_NOCTTY | O_SYNC) < 0) {
        perror("Failed to set fd to O_RDWR|O_NONBLOCK|O_NOCTTY | O_SYNC");
        return false;
    }
    return true;
}

static bool setBlocking(int fd) {
    //switch back to nonblocking
    if(fcntl(fd, F_SETFL, O_RDWR| O_NOCTTY | O_SYNC) < 0) {
        perror("Failed to set fd to O_RDWR| O_NOCTTY | O_SYNC");
        return false;
    }
    return true;
}

bool URG::startAcquisition(int nScans, int startStep, int endStep, int scanInterval, int clusterCount){
    // switch on the laser
    if (!URG::simpleCommand(URG_BM))
        return false;

    if (startStep == -1)
        startStep = m_info.stepMin;
    if (endStep == -1)
        endStep = m_info.stepMax;

    cerr << "switched on laser ..." << endl;

    char command[1024];
    sprintf (command, "MD%04d%04d%02d%1d%02d", startStep, endStep, clusterCount, scanInterval, nScans);
    if (strlen(command) != MDMS_COMMAND_LENGTH)
    {
        cerr << "MDMS_COMMAND_LENGTH does not match the size of the command we are sending. Fix the code." << endl;
        return error(INTERNAL_ERROR);
    }

    SimpleCommand cmd = { command, 0 };
    if (!simpleCommand(cmd))
    {
        if (m_error != UNKNOWN)
            return false;

        int status = atoi(m_last_status);
        if (status > 0 && status < 8)
            return error(BAD_COMMAND);

        // Still unknown error ...
        return false;
    }

    return true;
}

bool URG::readRanges(DFKI::LaserReadings& range, int timeout)
{
    if (timeout == -1)
        timeout = 2 * 60000 / m_info.motorSpeed;

    char const* expected_cmds[] = { "MD", "MS", 0 };

    char buffer[MAX_PACKET_SIZE];
    int packet_size = readAnswer(buffer, MAX_PACKET_SIZE, expected_cmds, timeout);
    if (packet_size < 0)
        return false;

    timeval tv;
    gettimeofday(&tv, 0);
    range.stamp.seconds = tv.tv_sec;
    range.stamp.microseconds = tv.tv_usec;

    buffer[packet_size] = 0;

    // Check status. Use parseErrorCode for standard error codes, and then do
    // our own for MDMS-specific ones.
    char const* status_code = buffer + MDMS_COMMAND_LENGTH + 1;
    if (!parseErrorCode(status_code, 0))
    {
        if (m_error != UNKNOWN)
            return false;

        int status = atoi(string(status_code, 2).c_str());
        if (status > 0 && status < 8)
            return error(BAD_COMMAND);
        if (status >= 50)
            return error(HARDWARE_FAILURE);

        // Mmmm .. still an unknown error ...
        return false;
    }

    // Read step setup from the command echo
    int startStep, endStep, clusterCount;
    { char v[5];
        v[4]=0;
        strncpy(v,buffer + 2,4);  startStep = atoi(v);
        strncpy(v,buffer + 6,4);  endStep   = atoi(v);
        v[2]=0;
        strncpy(v,buffer + 10,2); clusterCount = atoi(v);
    }
    size_t const expected_count = (endStep - startStep + 1) / clusterCount;
    range.min   = startStep / clusterCount;
    range.count = expected_count;
    range.resolution = m_info.resolution / clusterCount;
    range.speed = 60000000 / (m_info.motorSpeed * m_info.resolution);

    char const* timestamp = buffer + MDMS_COMMAND_LENGTH + 5;
    int device_timestamp = parseInt(4, timestamp);
    if (!timestamp)
        return error(BAD_REPLY);

    char const* data = timestamp + 2;
    for (size_t i = 0; i < expected_count; ++i) {
        if (data == 0)
        {
            cerr << "read " << i << " ranges, expected " << expected_count << endl;
            return error(INCONSISTEN_RANGE_COUNT);
        }

        range.ranges[i] = parseInt(3, data);
    }
    if (data && data[1] != '\n')
    {
        cerr << "expected " << expected_count << " ranges, but got more" << endl;
        cerr << printable_com(data) << endl;
    }

    return true;
}

bool URG::stopAcquisition() {
    return URG::simpleCommand(URG_QUIT);
}

URG::~URG() {
    if (fd != -1)
	close();
}
bool URG::close() {
    stopAcquisition();
    setBaudrate(19200);
    ::close(fd);
    fd = -1;
    return true;
}

bool URG::open(const char* filename){
    //first open nonblockiong and test baudrate
    struct termios tio;

    fd = ::open(filename, O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK );
    if (fd < 0){
        printf("Failed to open %s: ", filename);
        perror("");
        return false;
    }

    tcgetattr(fd,&tio);

    tio.c_cflag=(tio.c_cflag & ~CSIZE) | CS8; // data bits = 8bit

    tio.c_iflag&= ~( BRKINT | ICRNL | ISTRIP );
    tio.c_iflag&= ~ IXON;    // no XON/XOFF
    tio.c_cflag&= ~PARENB;   // no parity
#ifndef LINUX
    tio.c_cflag&= ~CRTSCTS;  // no CTS/RTS
    tio.c_cflag&= ~CSTOPB;   // stop bit = 1bit
#endif

#ifdef CYGWIN
    tio.c_cc[VMIN] = 1;
    tio.c_cc[VTIME] = 1;
#endif

    // Other
    tio.c_lflag &= ~( ISIG | ICANON | ECHO );

    // Commit
    if(tcsetattr(fd,TCSADRAIN,&tio)!=0)
        return false;

    // Reset the scanner
    if (! fullReset())
        return false;

    if (! setBaudrate(baudrate))
        return false;
    if (! readInfo())
        return false;

    // m_info.stepCount is not the same value ... I don't actually know what it is ...
    size_t stepCount = m_info.stepMax - m_info.stepMin;
    if (stepCount > DFKI_LASER_MAX_READINGS)
    {
        cerr << "device is capable of " << stepCount << " steps, but DFKI_LASER_MAX_READINGS == " << DFKI_LASER_MAX_READINGS << ". Update the code" << endl;
        return error(INTERNAL_ERROR);
    }

    return true;
}

std::ostream& operator << (ostream& io, URG::DeviceInfo info)
{

    float deg_per_step = 360.0 / info.resolution;
    io << "Device: " << info.values["MODL"] << " (S/N " << info.values["SERI"] << ")\n"
        << "  firmware:    " << info.values["FIRM"] << "\n"
        << "  scan range:  [" << info.dMin << ", " << info.dMax << "]" << "\n"
        << "  resolution:  :" << info.resolution << " steps, " << deg_per_step << " degree per step\n"
        << "  scan region: [" << info.stepMin << ", " << info.stepMax << "]" << " steps, [" << info.stepMin * deg_per_step << ", " << info.stepMax * deg_per_step << "] deg\n"
        << "  scan period: " << 60000 / info.motorSpeed << "ms" << endl;

    return io;
}


