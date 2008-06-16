#include "hokuyourg.hh"
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
  if (!urg.open(argv[1]))
  {
      cerr << "cannot open device: " << urg.errorString() << endl;
      perror("errno is");
      return 1;
  }
  cout << urg.getInfo() << endl;

  if (!urg.setBaudrate(115200))
  {
      cerr << "cannot set baudrate: " << urg.errorString() << endl;
      perror("errno is");
      return 1;
  }

  if (!urg.startAcquisition(0, 0, 768))
  {
      cerr << "cannot start acquisition: " << urg.errorString() << endl;
      perror("errno is");
      return 1;
  }

  URG::RangeReading ranges;
  timeval reftime;
  gettimeofday(&reftime, 0);
  for (int i = 0; i < 20; ++i)
  {
      if (!urg.readRanges(ranges, 1000))
      {
          cerr << "failed to read ranges: " << urg.errorString() << endl;
          perror("errno is");
          return 1;
      }

      int dt = (ranges.cpu_timestamp.tv_sec - reftime.tv_sec) * 1000 + ranges.cpu_timestamp.tv_usec / 1000;
      cerr << i << " " << ranges.device_timestamp << " " << dt << endl;
  }

  urg.stopAcquisition();
  for (int i = 0; i < 20; ++i)
  {
      if (!urg.readRanges(ranges, 1000))
      {
          cerr << "the device did stop scanning ..." << endl;
          break;
      }
  }

  urg.close();
}

