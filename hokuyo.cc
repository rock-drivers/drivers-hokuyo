#include "hokuyo.hh"

#include <map>
#include <string>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <algorithm>

#include <math.h>
#include <termio.h>
#include <sys/types.h>
#include <sys/time.h>
#include <time.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>

using namespace std;

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

    { URG::DUPLICATE,                  "device resent a set of ranges" },

    { URG::UNKNOWN_DEVICE_VERSION,     "the returned device version is not known to the driver" },
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
    { "03", URG::OK }, // already at the given bit rate
    { "04", URG::NON_APPLICABLE }, // no bit rate setting (i.e. non-serial interface)
    { 0, URG::END }
};

static URG::StatusCode URG_TMx_STATUS_CODES[] = {
    { "01", URG::BAD_COMMAND },
    { "02", URG::OK }, //adjust on request while already on
    { "03", URG::OK }, //adjust off request while already off
    { "04", URG::BAD_STATE }, //time request while adjust off
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
static URG::SimpleCommand URG_TM0 = { "TM0", URG_TMx_STATUS_CODES };
static URG::SimpleCommand URG_TM2 = { "TM2", URG_TMx_STATUS_CODES };

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
    : Driver(MAX_PACKET_SIZE)
    , baudrate(19200)
    , m_error(OK)
    , last_device_timestamp(-1)
    , sample_count(0)
{
    m_last_status[0] = 
        m_last_status[1] = 
        m_last_status[2] = 0;
}
URG::~URG()
{
    if (isValid())
        close();
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
    snprintf(buffer, MAX_PACKET_SIZE, "\n%s\n", string);
    size_t cmd_size = strlen(buffer);
    try
    {
        Driver::writePacket(reinterpret_cast<uint8_t*>(buffer), cmd_size, timeout);
        return true;
    }
    catch(iodrivers_base::TimeoutError)  { return error(WRITE_TIMEOUT); }
    catch(...) { return error(WRITE_FAILED); }
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
    {
        std::cerr << fields["STAT"] << std::endl;
        return error(BAD_STATE);
    }

    string version = string(fields["MODL"], 0, 8);
    if (version == "UTM-30LX")
        m_info.version = DeviceInfo::UTM30LX;
    else if (version == "URG-04LX")
        m_info.version = DeviceInfo::URG04LX;
    else
    {
        m_info.version = DeviceInfo::UNKNOWN;
        std::cerr << "driver returned version '" << version << "', which is not known to the driver" << std::endl;
        return error(UNKNOWN_DEVICE_VERSION);
    }

    m_info.values     = fields;
    m_info.dMin       = atoi(fields["DMIN"].c_str());
    m_info.dMax       = atoi(fields["DMAX"].c_str());
    m_info.resolution = atoi(fields["ARES"].c_str());
    m_info.stepMin    = atoi(fields["AMIN"].c_str());
    m_info.stepMax    = atoi(fields["AMAX"].c_str());
    m_info.stepFront  = atoi(fields["AFRT"].c_str());
    m_info.motorSpeed = static_cast<float>(atoi(fields["SCAN"].c_str())) / 60;
    return true;
}

int URG::readAnswer(char* buffer, size_t buffer_size, char const* expected_cmd, int timeout)
{
    char const* cmds[] = { expected_cmd, 0 };
    return readAnswer(buffer, buffer_size, cmds, timeout);
}

int URG::readAnswer(char* buffer, size_t buffer_size, char const** expected_cmds, int timeout)
{
    base::Time start_time = base::Time::now();
    try
    {
        while(true)
        {
            size_t packet_size = readPacket(reinterpret_cast<uint8_t*>(buffer), buffer_size, timeout);

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

            if ((base::Time::now() - start_time).toMilliseconds() > timeout)
            {
                error(READ_TIMEOUT);
                return -1;
            }
        }
    }
    catch(iodrivers_base::TimeoutError) { error(READ_TIMEOUT); return -1; }
    catch(std::exception &e) { cerr << e.what() << endl; error(READ_FAILED); return -1; }
}

int URG::extractPacket(uint8_t const* buffer, size_t buffer_size) const {
    for (size_t i = 1; i < buffer_size; ++i)
    {
        if (buffer[i - 1] == '\n' && buffer[i] == '\n')
        {
            //std::cerr << "R " << string(buffer, buffer + i);
            return i + 1;
        }
    }

    if( buffer_size >= MAX_PACKET_SIZE )
	return -buffer_size;

    return 0;
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

bool URG::timeCommand(int &device_timestamp, int timeout) {
    const char * cmd = "TM1";
    char buf[MAX_PACKET_SIZE];
    int cmd_size = 3;
    if (!write(cmd))
        return false;

    int packet_size = readAnswer(buf, MAX_PACKET_SIZE, cmd, timeout);
    if (packet_size < 0) {
        return false;
    }

    char const *status_code = buf + cmd_size + 1;
    if (!parseErrorCode(status_code, 0))
    {
        if (m_error != UNKNOWN)
            return false;

        int status = atoi(string(status_code, 2).c_str());
        if (status > 0 && status < 5)
            return error(BAD_COMMAND);

        // Mmmm .. still an unknown error ...
        return false;
    }

    char const* timestamp = buf + cmd_size + 5;

    //DON'T remove line, parseInt advances on the stream
    device_timestamp = parseInt(4, timestamp);
    if (!timestamp) {
	return error(BAD_REPLY);
    }

    return true;
}

bool URG::fullReset() {
    cerr << "Resetting scanner..." << flush;
    size_t baudrates[]={19200, 57600, 115200};
    const int baudrates_count = 3;

    int i;
    for (i=0; i < baudrates_count; i++){
        if (!setSerialBaudrate(baudrates[i]))
            return error(URG::BAD_HOST_RATE);;

	// before doing anything send a quit command, since
	// the scanner could still be in continous mode.
	// If this is the case, we will also not get a response here,
	// so don't wait for it.
	simpleCommand(URG_QUIT, 0);

	//we may be in adjust on mode, so send the command, but don't
	//check the result, since we may be in non-SCIP2 mode
	simpleCommand(URG_TM2, 0);

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

    if (!setSerialBaudrate(19200))
        return error(URG::BAD_HOST_RATE);;

    // Set baud rate to default
    cerr << " done" << endl;
    baudrate = 19200;
 
    // Have to wait after the reset, in order to get the device up and working
    timespec tv = { 1, 0 };
    nanosleep(&tv, &tv);

    // now try for synchronisation
    if (!simpleCommand(URG_TM0, 1000)) {
	simpleCommand(URG_TM2, 0);
	return false;
    }
    int ts;
    base::Time t1 = base::Time::now();
    if (!timeCommand(ts, 1000)) {
	simpleCommand(URG_TM2, 0);
	return false;
    }
    base::Time t2 = base::Time::now();
    if (!simpleCommand(URG_TM2, 1000))
	return false;

    device_time_offset = t1/2+t2/2-base::Time::fromMicroseconds(ts*1000);

    return true;
}

bool URG::setBaudrate(int brate){
    char cmd[MAX_PACKET_SIZE];

    if (!isValid())
    {
        this->baudrate = brate;
        return true;
    }

    // switch to current baudrate
    if(! setSerialBaudrate(baudrate))
        return error(URG::BAD_HOST_RATE);;

    sprintf(cmd, "SS%06d", brate);
    SimpleCommand cmd_obj = { cmd, URG_SS_STATUS_CODES };
    if (URG::simpleCommand(cmd_obj))
    {
        if (!setSerialBaudrate(brate))
            return error(URG::BAD_HOST_RATE);;

        baudrate = brate;
        
        // Have to wait after the reset, in order to get the communication link reset
        timespec tv = { 1, 0 };
        nanosleep(&tv, &tv);
        return true;
    }
    else if (error() == URG::NON_APPLICABLE)
        this->baudrate = brate;

    if (!URG::readInfo())
        return false;
    return baudrate == brate;
}

bool URG::startAcquisition(int nScans, int startStep, int endStep, int scanInterval, int clusterCount, bool includeRemission ){
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
    // set command to ME to also obtain remission values
    // everything else stays the same
    if( includeRemission )
	command[1] = 'E';

    if ((int)strlen(command) != MDMS_COMMAND_LENGTH)
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

    // setup the lookup table for normalising the remission values
    if( includeRemission )
	initRemissionLookup();
    
    return true;
}

bool URG::readRanges(base::samples::LaserScan& range, int timeout)
{
    // calculate the timeout to be three cycles
    // one measurement should usually appear after one cycle,
    // however, for extended measurements with remission, 
    // two cycles are needed for transmission. 
    // a timeout of 5 cycles should be safe for all cases
    if (timeout == -1)
        timeout = 5 * 1000 / m_info.motorSpeed;

    char const* expected_cmds[] = { "MD", "ME", "MS", 0 };

    char buffer[MAX_PACKET_SIZE];
    int packet_size = readAnswer(buffer, MAX_PACKET_SIZE, expected_cmds, timeout);
    if (packet_size < 0)
        return false;

    //range.time = base::Time::now();
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

    // Check command echo for ME command
    bool includeRemission = false;
    if( *(buffer + 1) == 'E' )
	includeRemission = true;

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

    range.start_angle = (startStep - m_info.stepFront) * 2.0 * M_PI / m_info.resolution;
    range.angular_resolution = 2.0 * M_PI / m_info.resolution * clusterCount;
    range.speed = m_info.motorSpeed * 2.0 * M_PI;
    range.minRange = m_info.dMin;
    range.maxRange = m_info.dMax;

    // read timestamp
    char const* timestamp = buffer + MDMS_COMMAND_LENGTH + 5;

    //DON'T remove line, parseInt advances on the stream
    int device_timestamp = parseInt(4, timestamp);
    if (!timestamp)
        return error(BAD_REPLY);

    if (device_timestamp == last_device_timestamp)
	return error(DUPLICATE);

    //wraparound of the internal timer
    if (device_timestamp < last_device_timestamp)
	device_time_offset = device_time_offset + base::Time::fromMicroseconds(1 << 24);

    // subtract 3.1 ms for the difference between "back of the scanner"
    // and measurement 0
    range.time = device_time_offset+base::Time::fromMicroseconds(device_timestamp*1000-3100);

    //period of the device
    base::Time period = base::Time::fromSeconds(1.0 / (range.speed / (M_PI * 2.0)));
    
    //compute the sample counter
    int sample_count_diff = 0;
    if(last_sample_time != base::Time())
    {
	sample_count_diff = round((range.time.toSeconds() - last_sample_time.toSeconds()) / period.toSeconds());
    }    
    sample_count += sample_count_diff;

    last_sample_time = range.time;
    last_device_timestamp = device_timestamp;

    {
	// check that the buffer has the right size for the expected readings
	int data_size = packet_size - MDMS_COMMAND_LENGTH - 11 - 3;
	int expected_size = (expected_count * 3 * (includeRemission+1));
	expected_size = expected_size + (expected_size/64)*2;

	if( data_size != expected_size )
	{
	    cerr << "inconsisted buffer size, expected " << expected_size << " got " << data_size << endl;
	    if( expected_size < data_size )
		cerr << "remaining bytes in buffer: " << printable_com(buffer+MDMS_COMMAND_LENGTH+11+3+expected_size) << endl;
	    return error(INCONSISTEN_RANGE_COUNT);
	}
    }

    range.ranges.resize(expected_count);
    if( includeRemission )
	range.remission.resize(expected_count);

    char const* data = buffer + MDMS_COMMAND_LENGTH + 11;

    for (size_t i = 0; i < expected_count; ++i) 
    {
	int range_value = parseInt(3, data);
        range.ranges[i] = range_value;
	if( includeRemission )
	    range.remission[i] = normaliseRemission( parseInt(3, data), range_value );

        if (range.ranges[i] < (size_t)m_info.dMin)
        {
            // an error has occured. In the case of the URG-04, classify them
            if (m_info.version == DeviceInfo::URG04LX)
            {
                if (range.ranges[i] == 0)
                    range.ranges[i] = base::samples::TOO_FAR;
                else
                    range.ranges[i] = base::samples::MEASUREMENT_ERROR;
            }
            else if (m_info.version == DeviceInfo::UTM30LX)
            {
                if (range.ranges[i] == 4)
                    range.ranges[i] = base::samples::TOO_FAR;
                else if (range.ranges[i] > 4)
                    range.ranges[i] = base::samples::OTHER_RANGE_ERRORS;
            }
        }
    }
    if (data && data[1] != '\n')
    {
        cerr << "expected " << expected_count << " ranges, but got more" << endl;
        cerr << "remaining bytes in buffer: " << printable_com(data) << endl;
        return error(BAD_REPLY);
    }

    return true;
}

int URG::getPacketCounter()
{
    return sample_count;
}

bool URG::stopAcquisition() {
    return URG::simpleCommand(URG_QUIT);
}

bool URG::close() {
    stopAcquisition();
    setBaudrate(19200);
    Driver::close();
    return true;
}

bool URG::open(std::string const& filename){
    if (! Driver::openSerial(filename, 19200))
        return false;

    int desired_baudrate = baudrate;

    // Reset the scanner
    if (! fullReset())
        return false;

    if (desired_baudrate != baudrate)
    {
        if (! setBaudrate(desired_baudrate))
            return false;
    }
    if (! readInfo())
        return false;

    return true;
}

void URG::initRemissionLookup() 
{
    // we assume the underlying model to be
    // remission_value = a * b^range, where 
    // a is the factor that we are interested in
    // since 1/b^range is quite expensive to do
    // for each point, we do it in a lookup table

    // const values where evaluated from data give
    // by hokuyo in document "SCIP2.0 SPECIAL COMMANDS"
    //
    // TODO: This model is crap!!! But it seems that the data
    // cannot be normalised by the distance alone.
    // leave this till later, and return unnormalised remission 
    // values for now.
/*
    const float b = 0.99989985;

    remission_lookup.clear();
    remission_lookup.resize( MAX_RANGE_READING );
    for(int i=0;i<MAX_RANGE_READING;i++) 
    {
	remission_lookup[i] = 1.0/pow(b, i);
    }
*/
}

float URG::normaliseRemission( int raw, int range )
{
    return raw;
//    range = std::min<int>( MAX_RANGE_READING-1, range );
//    return remission_lookup[range] * raw;
}


std::ostream& operator << (ostream& io, URG::DeviceInfo info)
{

    float deg_per_step = 360.0 / info.resolution;

    io << "Device: " << info.values["MODL"] << " (S/N " << info.values["SERI"] << ")\n"
        << "  firmware:    " << info.values["FIRM"] << "\n"
        << "  scan range:  [" << info.dMin << ", " << info.dMax << "]" << "\n"
        << "  resolution:  :" << info.resolution << " steps, " << deg_per_step << " degree per step\n"
        << "  scan region: " << info.stepMax - info.stepMin + 1 << " steps, " << (info.stepMax - info.stepMin + 1) * deg_per_step << " deg\n"
        << "  scan period: " << 1000 / info.motorSpeed << "ms" << endl;

    return io;
}


