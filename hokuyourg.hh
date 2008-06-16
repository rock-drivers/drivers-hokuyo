#ifndef URG_H
#define URG_H

#include <map>
#include <string>
#include <iosfwd>
#include <sys/types.h>

class URG {
public:
  struct StatusCode;
  struct SimpleCommand;

  static const int MAX_BEAMS             = 768;
  static const int MAX_PACKET_SIZE       = 8192;
  static const int MDMS_COMMAND_LENGTH   = 15;
  static const int MDMS_TIMESTAMP_OFFSET = MDMS_COMMAND_LENGTH + 5;
  static const int MDMS_DATA_OFFSET      = MDMS_TIMESTAMP_OFFSET + 6;

  enum RANGE_ERRORS {
      TOO_FAR = 0,
      LOW_INTENSITY0 = 1,
      LOW_INTENSITY1 = 2,
      LOW_INTENSITY2 = 3,
      LOW_INTENSITY3 = 4,
      LOW_INTENSITY4 = 5,
      NEXT_PREVIOUS_ERROR = 7,
      INTENSITY_DIFFERENCE = 8,
      TWO_PREVIOUS_ERROR = 9,
      STRONG_REFLECTIVE_OBJECT = 18,
      NON_MEASURABLE_STEP = 19
  };
  struct RangeReading {
      int device_timestamp;
      timeval cpu_timestamp;

      int rangeCount;
      unsigned short ranges[MAX_BEAMS];
      unsigned short startStep, endStep, clusterCount;
  };

  struct DeviceInfo {
      DeviceInfo()
          : dMin(0), dMax(0), resolution(0), stepMin(0), stepMax(0), stepFront(0), motorSpeed(0) { }

      std::map<std::string, std::string> values;
      int dMin;       /// the minimal measurable distance
      int dMax;       /// the biggest measurable distance
      int resolution; /// how much steps in one full turn
      int stepMin;    /// the first measurable step
      int stepMax;    /// the last measurable step
      int stepFront;  /// the step number in front of the device
      int motorSpeed; /// Motor speed in turn per minute
  };

  enum ERROR_CODES {
    OK                  = 0,
    
    INTERNAL_COM_FAILED    = 2,
    DEVICE_BUFFER_OVERFLOW = 3,
    BAD_COMMAND         = 4,
    STRING_TOO_LONG     = 5,
    BAD_STRING          = 6,
    FIRMWARE_UPDATE     = 7,
    LASER_MALFUNCTION   = 8,
    HARDWARE_FAILURE    = 9,
    
    BAD_REPLY                   = 100,
    DRIVER_BUFFER_OVERFLOW      = 121,
    PROVIDED_BUFFER_TOO_SMALL   = 122,
    BAD_STATE                   = 123,
    READ_FAILED                 = 124,
    BAD_RATE                    = 125,
    WRITE_FAILED                = 126,
    READ_TIMEOUT                = 127,
    NOT_SCIP2_CAPABLE           = 128,
    BAD_HOST_RATE               = 129,
    INCONSISTEN_RANGE_COUNT     = 130,
    WRITE_TIMEOUT               = 131,

    UNKNOWN                     = 200,
    INTERNAL_ERROR              = 201,

    END				= 500
  };

private:
  int fd;
  int baudrate;
  char        m_last_status[3];
  ERROR_CODES m_error;
  DeviceInfo  m_info;

  // Device capabilities
  /// Used for reading
  char internal_buffer[MAX_PACKET_SIZE];
  size_t internal_buffer_size;

  /** \overloaded */
  int readAnswer(char* buffer, size_t buffer_size, char const* expected_cmd, int timeout = 1000);

  /** Waits for the device returning an answer for the given commands. The
   * answer packet is returned into \c buffer. \c timeout is how much time we
   * wait for the answer, in milliseconds. +expected_cmd+ is a null-terminated
   * array of commands that are waited for. */
  int readAnswer(char* buffer, size_t buffer_size, char const** expected_cmds, int timeout = 1000);

  bool readInfo();
  bool simpleCommand(SimpleCommand const& cmd, int timeout = 1000);
  int read(char* buffer, size_t buffer_size);
  /** Write the command defined in \c string. This command will be internally
   * enclosed in \n, so it does not need to be done already.
   *
   * \c timeout is how much time, in milliseconds, the driver can wait for the
   * file descriptor to be available for writing
   */
  bool write(char const* string, int timeout = 1000);
  static void parseReading(RangeReading& r, char* buffer);

  bool error(ERROR_CODES code);
  bool parseErrorCode(char const* code, StatusCode const* specific_codes);
  bool infoCommand(std::map<std::string, std::string>& result, char const* cmd, bool scip1 = false);

public:
  URG();
  ~URG();

  /** Open the device, reset it and read device information. After this call,
   * the baud rate is 19200 and the device is not scanning */
  bool open(char const* filename);
  bool reset();
  /** Performs a full reset of the device. After this call, the device baud
   * rate is 19200 and it is not scanning */
  bool fullReset();
  /** Closes the device */
  bool close();
  DeviceInfo getInfo() const { return m_info; };

  /** Start continuous acquisition. Call readRanges() to get a frame.
   *
   * @arg nFrames the count of frames to acquire. Set to 0 to do continuous acquisition 
   * @arg scanInterval the count of scans to ignore between two reported scans. Set to 0 to do continuous acquisition 
   * @arg startStep the step at which to start reading ranges. See the device information to get minimal and maximal readable steps 
   * @arg endStep the step at which to stop reading ranges. See the device information to get minimal and maximal readable steps
   * @arg clusterCount how many ranges the device should merge into one reported range
   */
  bool startAcquisition(int nScans, int startStep, int endStep, int scanInterval = 0, int clusterCount = 1);
  /** Stop continuous acquisition */
  bool stopAcquisition();
  /** Gets a range reading and decodes it into \c range. If timeout is
   * positive, it is how much time, in milliseconds, we should wait for an
   * answer. If it is -1, it is set to twice the scan duration */
  bool readRanges(RangeReading& range, int timeout = -1);
  /** Sets the device baudrate (only possible for serial devices). +brate+ is
   * the baud rate in bps, can be one of 19200, 57600 and 115200 */
  bool setBaudrate(int brate);

  /** Returns the last error code registered by the driver */
  ERROR_CODES error() const { return m_error; }
  /** Returns the string describing the given error code */
  static char const* errorString(int error_code);
  /** Returns a string describing the current error */
  char const* errorString() const { return errorString(m_error); }
};

std::ostream& operator << (std::ostream& io, URG::DeviceInfo info);

#endif

