#ifndef URG_H
#define URG_H

#include <map>
#include <string>
#include <iosfwd>
#include <sys/types.h>
#include <dfki/laser_readings.h>
#include <iodrivers_base.hh>

/** This class implements a driver for the Hokuyo laser range finders.
 *
 * Note on error handling: all methods return a boolean argument. If the
 * returned value is true, everything is fine.  Otherwise, an error occured and
 * the exact error code is returned by URG::error(). URG::ERROR_CODES lists all
 * error codes and URG::errorString(int) can return a string describing the
 * exact error.
 */
class URG : public IODriver {
public:
  struct StatusCode;
  struct SimpleCommand;

  /** For internal use */
  static const int MDMS_COMMAND_LENGTH   = 15;
  /** For internal use */
  static const int MDMS_TIMESTAMP_OFFSET = MDMS_COMMAND_LENGTH + 5;
  /** For internal use */
  static const int MDMS_DATA_OFFSET      = MDMS_TIMESTAMP_OFFSET + 6;

  /** Special values for the ranges. If a range has one of these values, then
   * it is not valid and the value declares what is going on */
  enum RANGE_ERRORS {
      TOO_FAR            = 1, // too far
      TOO_NEAR           = 2,
      MEASUREMENT_ERROR  = 3,
      OTHER_RANGE_ERRORS = 4,
      MAX_RANGE_ERROR    = 4
  };

  /** This structure is returned by getInfo() to represent various informations
   * on the device.
   */
  struct DeviceInfo {
      DeviceInfo()
          : version(UNKNOWN), dMin(0), dMax(0), resolution(0), stepMin(0), stepMax(0), stepFront(0), motorSpeed(0) { }

      enum VERSIONS {
          URG04LX,
          UTM30LX,
          UNKNOWN
      };
      std::map<std::string, std::string> values;
      VERSIONS version;
      int dMin;       /// the minimal measurable distance
      int dMax;       /// the biggest measurable distance
      int resolution; /// how much steps in one full turn
      int stepMin;    /// the first measurable range
      int stepMax;    /// the last measurable step
      int stepFront;  /// the step number in front of the device
      int motorSpeed; /// Motor speed in turn per minute
  };

  /** All the possible error codes that can be returned by URG::error() */
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
    NON_APPLICABLE      = 10,
    
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
    UNKNOWN_DEVICE_VERSION      = 202,

    END				= 500
  };

private:
  static const int MAX_PACKET_SIZE = 4096;

  /** The baudrate */
  int         baudrate;
  char        m_last_status[3];
  /** The current error */
  ERROR_CODES m_error;
  /** The device info. This is read on initialization */
  DeviceInfo  m_info;

  /** \overloaded */
  int readAnswer(char* buffer, size_t buffer_size, char const* expected_cmd, int timeout = 1000);

  /** Waits for the device returning an answer for the given commands. The
   * answer packet is returned into \c buffer. \c timeout is how much time we
   * wait for the answer, in milliseconds. +expected_cmd+ is a null-terminated
   * array of commands that are waited for. */
  int readAnswer(char* buffer, size_t buffer_size, char const** expected_cmds, int timeout = 1000);

  bool readInfo();
  bool simpleCommand(SimpleCommand const& cmd, int timeout = 1000);
  /** Write the command defined in \c string. This command will be internally
   * enclosed in \n, so it does not need to be done already.
   *
   * \c timeout is how much time, in milliseconds, the driver can wait for the
   * file descriptor to be available for writing
   */
  bool write(char const* string, int timeout = 1000);

  bool error(ERROR_CODES code);
  bool parseErrorCode(char const* code, StatusCode const* specific_codes);
  bool infoCommand(std::map<std::string, std::string>& result, char const* cmd, bool scip1 = false);

protected:
  int extractPacket(uint8_t const* buffer, size_t buffer_size) const;

public:
  URG();
  ~URG();

  /** Open the device, reset it and read device information. After this call,
   * the baud rate is 19200 and the device is not scanning */
  bool open(std::string const& filename);
  /** Performs a full reset of the device. After this call, the device baud
   * rate is 19200 and it is not scanning */
  bool fullReset();
  /** Closes the device */
  bool close();
  /** Returns the device info structure. This does not access the device,
   * as the structure is read at initialization in open() */
  DeviceInfo getInfo() const { return m_info; };

  /** Start continuous acquisition. Call readRanges() to get a frame.
   *
   * @arg nFrames the count of frames to acquire
   * @arg startStep the step at which to start reading ranges. See the device
   *      information to get minimal and maximal readable steps. Set to -1 to use
   *      the first readable step defined for the device.
   * @arg endStep the step at which to stop reading ranges. See the device
   *      information to get minimal and maximal readable steps. Set to -1 to use
   *      the first readable step defined for the device.
   * @arg scanInterval the count of scans to ignore between two reported scans.
   * @arg clusterCount how many ranges the device should merge into one reported range
   */
  bool startAcquisition(int nScans, int startStep = -1, int endStep = -1, int scanInterval = 0, int clusterCount = 1);
  /** Stop continuous acquisition */
  bool stopAcquisition();
  /** Gets a range reading and decodes it into \c range. If timeout is
   * positive, it is how much time, in milliseconds, we should wait for an
   * answer. If it is -1, it is set to twice the scan duration */
  bool readRanges(DFKI::LaserReadings& range, int timeout = -1);
  /** Sets the device baudrate (only possible for serial devices). +brate+ is
   * the baud rate in bps, can be one of 19200, 57600 and 115200
   *
   * If the device is not open yet, this baud rate will be set on startup (i.e.
   * in open())
   */
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

