#include "hokuyo.hh"
#include <iostream>
#include <sys/time.h>
#include <time.h>

using namespace std;

int main (int argc, const char** argv){
  if (argc<2){
    printf( "Usage: urg_serial <device>\n");
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

  std::cout << urg.getInfo().values["SERI"];
}
