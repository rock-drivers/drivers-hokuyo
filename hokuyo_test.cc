#include "hokuyo.hh"
#include <iostream>
#include <sys/time.h>
#include <time.h>
#include <boost/lexical_cast.hpp>

using namespace std;

int main (int argc, const char** argv){
  if (argc<2){
    printf( "Usage: urg_test <device> [count]");
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
  URG::DeviceInfo device = urg.getInfo();
  cout << urg.getInfo() << endl;

  DFKI::LaserReadings ranges;

  int test = 1;
  size_t count = 20;
  if (argc >= 3)
      count = boost::lexical_cast<int>(argv[2]);

  if( count == 0 ) {
      if (!urg.startAcquisition(0, -1, -1, 0, 1, 1))
      {
	  cerr << "cannot start acquisition: " << urg.errorString() << endl;
	  perror("errno is");
	  return 1;
      }
      cout << "Now giving center reading values." << endl;

      while( true ) {
	  if (!urg.readRanges(ranges))
	  {
	      cerr << "failed to read ranges: " << urg.errorString() << endl;
	      perror("errno is");
	      return 1;
	  }

	  cout <<  ranges.ranges[540] << " " << ranges.remission[540] << "               " << endl;
      }
  }

  while(test >= 0) 
  {
      cout << endl << "Starting scan " << string(test?"including":"excluding") << " remission values" << endl << endl;

      if (!urg.startAcquisition(0, -1, -1, 0, 1, test))
      {
	  cerr << "cannot start acquisition: " << urg.errorString() << endl;
	  perror("errno is");
	  return 1;
      }

      DFKI::Time reftime = DFKI::Time::now();
      for (int i = 0; i < count; ++i)
      {
	  if (!urg.readRanges(ranges))
	  {
	      cerr << "failed to read ranges: " << urg.errorString() << endl;
	      perror("errno is");
	      return 1;
	  }

	  int too_far = 0;
	  int bad_ranges = 0;
	  int good_ranges = 0;
	  for (size_t range_idx = 0; range_idx < ranges.ranges.size(); ++range_idx)
	  {
	      unsigned int val = ranges.ranges[range_idx];
	      if (val <= URG::MAX_RANGE_ERROR)
	      {
		  if (val == URG::TOO_FAR)
		  {
		      too_far++;
		  }
		  else
		  {
		      bad_ranges++;
		  }

	      }
	      else
		  good_ranges++;
	  }

	  int dt = (ranges.stamp - reftime).toMilliseconds();
	  if (i != 0 && dt > 30*(test+1))
	      cerr << "!!!! LATENCY PROBLEM: " << dt << endl;
	  cout << i << " " << dt << " good=" << good_ranges << " too_far=" << too_far << " bad=" << bad_ranges << "\n";
	  reftime = ranges.stamp;
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

      test--;
  }

  urg.close();
}

