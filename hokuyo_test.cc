#include "hokuyo.hh"
#include <iostream>
#include <sys/time.h>
#include <time.h>

using namespace std;

int main (int argc, const char** argv){
  if (argc<2){
    printf( "Usage: urg_test <device> ");
    return 0;
  }

  URG urg;
  if (!urg.setBaudrate(115200))
  {
      cerr << "cannot set baudrate: " << urg.errorString() << endl;
      perror("errno is");
      return 1;
  }

  if (!urg.open(argv[1]))
  {
      cerr << "cannot open device: " << urg.errorString() << endl;
      perror("errno is");
      return 1;
  }
  cout << urg.getInfo() << endl;

  if (!urg.startAcquisition(0, -1, -1, 0, 2))
  {
      cerr << "cannot start acquisition: " << urg.errorString() << endl;
      perror("errno is");
      return 1;
  }

  DFKI::LaserReadings ranges;
  timeval reftime;
  gettimeofday(&reftime, 0);
  for (int i = 0; i < 20; ++i)
  {
      if (!urg.readRanges(ranges))
      {
          cerr << "failed to read ranges: " << urg.errorString() << endl;
          perror("errno is");
          return 1;
      }

      int too_far = 0;
      int bad_ranges;
      for (int range_idx = 0; range_idx < ranges.ranges.size(); ++range_idx)
      {
          unsigned int val = ranges.ranges[range_idx];
          if (val <= URG::MAX_RANGE_ERROR)
          {
              if (val == URG::TOO_FAR)
                  too_far++;
              else
                  bad_ranges++;
          }
      }

      int dt = (ranges.stamp.seconds - reftime.tv_sec) * 1000 + ranges.stamp.microseconds / 1000;
      cerr << i << " " << dt << "\n"
          << "  too far: " << too_far << "\n"
          << "  bad: " << bad_ranges << std::endl;

  }

  urg.stopAcquisition();
  for (int i = 0; i < 20; ++i)
  {
      if (!urg.readRanges(ranges))
      {
          cerr << "the device did stop scanning ..." << endl;
          break;
      }
  }

  urg.close();
}

