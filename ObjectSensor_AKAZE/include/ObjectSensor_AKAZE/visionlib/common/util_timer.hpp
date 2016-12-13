
//
// UTIL::Timer class object
//
// Jaeil Choi
// last modified in Jan, 2004
//
//
// Example:
//   UTIL::Timer timer;
//   cout << "DateTime : " << timer.LocalTime() << endl;
//   // for comparison, 'ctime()' returns "Mon Mar 10 19:22:37 2003\n"
//   timer.start();
//   double t  = timer.pause();
//   timer.resume();
//   double tt = timer.stop();
//


#ifndef UTIL_TIMER_HPP
#define UTIL_TIMER_HPP


#include <iostream>
#include <iomanip>
#include <ctime>
#include <cstdarg>

// ===================================================================

#ifdef WIN32
#include <Windows.h>
#define sleepd(sec)  Sleep((int)(sec*1000))
#else
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#define sleepd(sec)  \
  do { \
    struct timespec req, rem; \
    req.tv_sec = (time_t)((long)sec); \
    req.tv_nsec = (long)((sec - (double)req.tv_sec) * 1000000000); \
    nanosleep( &req, &rem ); \
  } while (0)
#endif

// ===================================================================

namespace UTIL {


// ===================================================================

typedef struct { char name[10]; double mx; double tt; int cnt; } sw_t;

class Timer
{
private:
  char    buffer1[160], buffer2[80];
  bool    running;
  double  accumulated_time;		// in sec
  clock_t last_start_time;

  int     sw_size;
  sw_t   *sw_data;

public:
  Timer() : running(false), accumulated_time(0), last_start_time(0), sw_size(0), sw_data(NULL) {}
  ~Timer() { if (sw_data) free(sw_data); }

  // -----------------------------------------------------------------
  // Local Date & Time
  // -----------------------------------------------------------------
public:
  inline int Year (struct tm *n) { return n->tm_year + 1900; }
  inline int YY   (struct tm *n) { return (n->tm_year < 100 ? n->tm_year : n->tm_year - 100);  }
  inline int Month(struct tm *n) { return n->tm_mon+1; }
  inline int Day  (struct tm *n) { return n->tm_mday;  }
  inline int WDay (struct tm *n) { return n->tm_wday;  }
  inline int YDay (struct tm *n) { return n->tm_yday;  }
  inline int Hour (struct tm *n) { return n->tm_hour;  }
  inline int Min  (struct tm *n) { return n->tm_min;   }
  inline int Sec  (struct tm *n) { return n->tm_sec;   }
  int MilliSec(void) {
#ifdef WIN32
    return 0;
#else
    struct timeval tv; struct timezone tz; gettimeofday(&tv, &tz); return (int)((tv.tv_usec%1000000)/1000);
#endif
  }

  char *LocalTime(int format=0, char *buffer=NULL) {
    time_t t = time(NULL);
    return FormatLocalTime(t, format, buffer);
  }

  char* FormatLocalTime(time_t t, int format, char *buffer=NULL) {
    struct tm *n = localtime(&t);
    if (buffer == NULL) buffer = this->buffer1;
    switch (format) {
    case 0:	// MM/DD/YYYY hh:mm:ss
      sprintf(buffer, "%02d/%02d/%04d %02d:%02d:%02d", Month(n), Day(n), Year(n), Hour(n), Min(n), Sec(n));
      break;
    case 1:	// MM/DD/YYYY
      sprintf(buffer, "%02d/%02d/%04d", Month(n), Day(n), Year(n));
      break;
    case 2:	// hh:mm:ss
      sprintf(buffer, "%02d:%02d:%02d", Hour(n), Min(n), Sec(n));
      break;
    case 3:	// hh:mm:ss MM/DD/YYYY
      sprintf(buffer, "%02d:%02d:%02d %02d/%02d/%04d", Hour(n), Min(n), Sec(n), Month(n), Day(n), Year(n));
      break;
    case 4:	// hh:mm, MM/DD/YYYY
      sprintf(buffer, "%02d:%02d, %02d/%02d/%04d", Hour(n), Min(n), Month(n), Day(n), Year(n));
      break;
    default:  return FormatLocalTime( t, 0, buffer );
    }
    return buffer;
  }

  char* getDateTimeString(char *buffer, char *head=NULL, char *tail=NULL) {
    time_t t = time(NULL);
    struct tm *n = localtime(&t);
    if (buffer == NULL) buffer = this->buffer1;
    sprintf(buffer, "%s%04d%02d%02d_%02d%02d_%02d%02d%s", (head ? head : ""),
	    Year(n), Month(n), Day(n), Hour(n), Min(n), Sec(n), MilliSec()/10,
	    (tail ? tail : ""));
    return buffer;
  }

  // -----------------------------------------------------------------
  // stopwatch (processor time in seconds)
  // -----------------------------------------------------------------
public:
  void start(const char* tname = NULL)  {
    if (tname) fprintf(stdout, "Timer Start   :       [%s] \n", tname);  // in sec
    running = true;
    accumulated_time = 0;
    last_start_time  = clock();
    //printf("started last_start_time=%g\n", (double)last_start_time);
  }
  double stop(const char* tname = NULL)  {
    // return the total accumulated time (sec) the timer has run
    if (running) {
      double elapsed = getElapsedTime();
      accumulated_time += elapsed;
      //printf("stopped accumulated=%g\n", accumulated_time);
      if (tname) fprintf(stdout, "Timer Stop    : %7.4f [%s]  total=%7.4f\n", elapsed, tname, accumulated_time);
    }
    running = false;       // change timer status to stopped
    return accumulated_time;
  }
  double pause(const char* tname = NULL) {
    // return elaspsed time (sec) since the the last resume/check/start
    if (!running) return 0;
    double elapsed = getElapsedTime();
    accumulated_time += elapsed;
    running = false;
    if (tname) fprintf(stdout, "Timer Paused  : %7.4f [%s] \n", elapsed, tname);
    return elapsed;
  }
  void resume(const char* tname = NULL)  {
    if (tname) fprintf(stdout, "Timer Resumed :       [%s] \n", tname);
    running = true;        // change timer status to running
    last_start_time = clock();  // Determine the start time;
  }
  double checkTime(const char* tname = NULL) {
    // return elaspsed time (sec) since the the last resume/check/start
    if (!running) return 0;
    double elapsed = getElapsedTime();
    accumulated_time += elapsed;
    //printf("  checked elapsed = %g  accumulated=%g\n", elapsed, accumulated_time);
    if (tname) fprintf(stdout, "Timer Checked : %7.4f [%s] \n", elapsed, tname);
    last_start_time = clock();  // Determine the start time;
    return elapsed;
  }

  inline double getElapsedTime(void)  {
    // Determine how much time (sec) has passed since the last resume/check/start
    clock_t curr_time = clock();
    double  elapsed = double(curr_time - last_start_time) / CLOCKS_PER_SEC;
    //printf("  elapsed = %g  curr=%g  last=%g\n", elapsed, (double)curr_time, (double)last_start_time);
    return elapsed;
  }
  double getTotalTime(void)  {
    if (running) return accumulated_time + getElapsedTime();
    else         return accumulated_time;
  }

  // -----------------------------------------------------------------
  // stopwatch with internal storage
  //   (to measure the performances of different modules in a program)
  // -----------------------------------------------------------------
public:
  void setupSWatch(int n, ... ) {
    // Set up a stopwatch to measure performance of parts of a program.
    //   Specify the number of fields 'n', and provide the (char*) names of the fields.
    if (sw_data) free(sw_data);
    sw_size = n;				//
    sw_data = (sw_t*)calloc( n, sizeof(sw_t) );	//
    va_list valist;				// copy names
    va_start( valist, n );
    for (int i=0; i<n; i++) {
      char *name = va_arg( valist, char* );  if (!name) break;
      strncpy( sw_data[i].name, name, 9 );
    }
    va_end( valist );
  }
  void startSWatch(void)  { if (!sw_data) return; start(); }
  void checkSWatch(int i) { if (!sw_data||i<0||i>=sw_size) return;   double et = checkTime(); sw_data[i].tt += et; /*if (et>sw_data[i].mx) sw_data[i].mx = et;*/ sw_data[i].cnt++; }
  void stopSWatch (int i) { if (!sw_data||i<0||i>=sw_size) stop(); else { double et = stop(); sw_data[i].tt += et; /*if (et>sw_data[i].mx) sw_data[i].mx = et;*/ sw_data[i].cnt++; } }
  void clearSWatch(void)  { for (int i=0; i<sw_size; i++) sw_data[i].mx = sw_data[i].tt = sw_data[i].cnt = 0; }
  void printSWatch(bool detail=true, int prec=3) {
    int  i, size=prec+4;
    char format[80];
    printf("SWatch ");		// field titles
    sprintf(format, "%%%ds ", size);
    for (i=0; i<sw_size; i++)  printf( format, sw_data[i].name );
    sprintf(format, "%%%d.%df ", size, prec);
//     printf("\n   Max ");		// maximum execution time
//     for (i=0; i<sw_size; i++)  printf( format, sw_data[i].mx );
    printf("\n   Avg ");		// average execution time
    for (i=0; i<sw_size; i++)  printf( format, (sw_data[i].cnt>0 ? sw_data[i].tt/sw_data[i].cnt : 0) );
    printf("\n   Sum ");		// total execution time
    for (i=0; i<sw_size; i++)  printf( format, sw_data[i].tt );
    printf("\n   Cnt "); 		// number of executions
    sprintf(format, "%%%dd ", size);
    for (i=0; i<sw_size; i++)  printf( format, sw_data[i].cnt );
    printf("\n");
    if (accumulated_time==0)
      printf("Warning: The measured time may not be accurate. (last accumulated_time==0)\n");
  }

  // -----------------------------------------------------------------
  // Measuring FPS (Frame Per Second) in iterative function calls
  // -----------------------------------------------------------------

  double getFPS(int n = 10) {
    static double history[50];
    static int    h_pos = 50;
    double sum = 0;
    if (--h_pos < 0) h_pos = 49;
    history[h_pos] = stop();
    // average over 'n' history records
    for (int i = 0; i < n; i++) sum += history[ (h_pos+i)%50 ];
    start();
    return (sum > 0 ? n/sum : 0.0);
  }

  // -----------------------------------------------------------------
  // Etc.
  // -----------------------------------------------------------------

  char* Format(double sec, int format, char *buffer=NULL) {
    if (buffer == NULL) buffer = this->buffer2;
    int h, m;
    m = (int)(sec / 60);  sec -= m * 60;
    switch (format) {
    case  0:					// HH:MM:SS
      h = (int)( m  / 60);  m   -= h * 60;
      sprintf(buffer, "%02d:%02d:%02.0f", h, m, sec);
      break;
    case  1: sprintf(buffer, "%02d:%02.0f", m, sec); break;  // MM:SS
    case  2: sprintf(buffer, "%02d:%02.2f", m, sec); break;  // MM:SS.00
    case  3: sprintf(buffer, "%d minutes %.0f seconds", m, sec); break;
    default: sprintf(buffer, "%02d:%02.0f", m, sec); break;
    }
    return buffer;
  }

};


}	// end of namespace UTIL

#endif  // UTIL_TIMER_HPP
