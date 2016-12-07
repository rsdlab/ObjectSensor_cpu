
//
// GUIH         : GUI in Header files
// GUIH::Config : configuration file processor
//
// (c) 2006  Jaeil Choi
// last modified in Aug, 2006
//
// --------------------------------------------------------------------------
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
// --------------------------------------------------------------------------
//
// GUIH is a very simple, light-weight and cross-platform Graphic User
//  Interface that can visualize images and OpenGL graphics with
//  keyboard and mouse interactions on MS Windows, Apple OS X, and Linux.
// It is intended as a tool for developers and researchers, who don't want
//  to spend time in designing/creating an user interface, but who want
//  all the features of a general user interface and the freedom to modify.
//  If you are not satisfied, like me, with GLUT, GLUI, Glow or FLTK,
//  GUIH might be the right tool for you.
// Key features include dialog windows, timers, default key bindings,
//  image capture, captions, mapping of mouse click to world-space coordinates,
//  and the built-in cameras for 2D/3D OpenGL.
// GUIH doesn't support a window with controls, except simple input dialogs.
//  For complex settings/configuration of an application, I recommend using
//  a configuration file, rather than a window full of controls.
//
// GUIH package consists of nine C++ header files:
//   guih_common.hpp     : the base window   GUIH::Window
//   guih_image.hpp      : image window      GUIH::ImageWindow
//   guih_video.hpp      : video window      GUIH::VideoWindow
//   guih_opengl2d.hpp   : 2D OpenGL Window  GUIH::OpenGL2DWindow
//   guih_opengl3d.hpp   : 3D OpenGL Window  GUIH::OpenGL3DWindow
//   guih_camera2d.hpp   : 2D camera for     GUIH::OpenGL2DWindow
//   guih_camera3d.hpp   : 3D camera for     GUIH::OpenGL3DWindow
//   guih_args.hpp       : command-line argument processing tool (GUIH::Args)
//   guih_config.hpp     : configuration file processing tool    (GUIH::Config)
// GUIH package defines five window classes:
//   Window              : the base window (Not for direct use!)
//    +- ImageWindow     : for image visualization
//    |   +- VideoWindow : for video visualization (with play/pause/seek)
//    +- OpenGL2DWindow  : for 2D OpenGL rendering (with built-in 2D camera)
//    +- OpenGL3DWindow  : for 3D OpenGL rendering (with built-in 3D camera)
// GUIH package defines two extra classes:
//   Args                : command-line argument processor
//   Config              : configuration file processor
//
// For usage, read the example code at the bottom of this file.
//


#ifndef GUIH_CONFIG_HPP
#define GUIH_CONFIG_HPP
#define USE_GUIH_CONFIG

#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <cfloat>

#include <stdlib.h>
#include <string.h>

namespace GUIH {


#define CFG_CHAR	1
#define CFG_INT		2
#define CFG_FLOAT	3
#define CFG_DOUBLE	4
#define CFG_STRING	5
#define CFG_BOOL	6
#define CFG_DIR		9

#define CFG_UNSET (-FLT_MIN)
#define CFG_NAME_MAX_LEN 22

typedef struct {
  int   sidx;				// section index
  char  key[CFG_NAME_MAX_LEN+1];
  bool  processed;
  bool  range_set;
  int   type;
  void  *p;
  double range[2];
} cfg_t;


class Config {
private:
  int    nsections;				//
  char   sections[40][CFG_NAME_MAX_LEN+1];	//
  cfg_t *keys;					// list of keys
  int    nkeys;					// number of keys

public:
  Config() : nsections(1), keys(NULL), nkeys(0) { sections[0][0] = '\0'; } // global
  ~Config() { clear(); }
  void clear(void) { if (keys) free(keys); keys=NULL; nkeys=0; nsections=1; }

public:
  int set(char *section, char *key, int type, void *p, double defaultv=CFG_UNSET) {
    return set(section, key, type, p, defaultv, CFG_UNSET, CFG_UNSET);
  }
  int set(char *section, char *key, int type, void *p, int defaultv) {
    return set(section, key, type, p, (double)defaultv, CFG_UNSET, CFG_UNSET);
  }
  int set(char *section, char *key, int type, void *p, char *defaultv) {
    return set(section, key, type, p, (double)(int)defaultv, CFG_UNSET, CFG_UNSET);
  }
  int set(char *section, char *key, int type, void *p, double defaultv, double min, double max) {
    int sidx = findSection(section);		// create the section, if necessary
    if (sidx < 0 && nsections < 40-1) {
      strncpy( sections[nsections], section, CFG_NAME_MAX_LEN );
      sections[nsections][CFG_NAME_MAX_LEN] = '\0';
      sidx = nsections;
      nsections++;
    }
    int kidx = findKey(sidx, key, false);	// create the key
    if (kidx < 0) {
      keys = (cfg_t*)realloc(keys, (nkeys+1) * sizeof(cfg_t));
      kidx = nkeys++;
      keys[kidx].processed = false;
      keys[kidx].sidx = sidx;
      strncpy(keys[kidx].key, key, CFG_NAME_MAX_LEN);	// copy the key
      keys[kidx].key[CFG_NAME_MAX_LEN] = '\0';
    }
    keys[kidx].type = type;
    keys[kidx].p = p;
    keys[kidx].range_set = false;
    keys[kidx].range[0] = min;  keys[kidx].range[1] = max;
    // set default values
    if        (type == CFG_BOOL) {
      *((bool*)(keys[kidx].p)) = (defaultv == 0 || defaultv == CFG_UNSET ? false : true);
    } else if (type == CFG_STRING || type == CFG_DIR) {
      if (defaultv == CFG_UNSET) strcpy((char*)(keys[kidx].p), "");
      else strcpy((char*)(keys[kidx].p), (char*)(int)defaultv);
      if (type == CFG_DIR) {
	char *str = (char*)(keys[kidx].p);
	int   len = (int)strlen(str);
	if (len > 0 && str[len-1] == '/') str[len-1] = '\0';
      }
    } else {
      switch (keys[kidx].type) {
      case CFG_INT:    *((int*)   (keys[kidx].p)) = (int)   defaultv; break;
      case CFG_FLOAT:  *((float*) (keys[kidx].p)) = (float) defaultv; break;
      case CFG_DOUBLE: *((double*)(keys[kidx].p)) = (double)defaultv; break;
      case CFG_CHAR:   *((char*)  (keys[kidx].p)) = (char)  defaultv; break;
      }
      if (min != CFG_UNSET || max != CFG_UNSET) keys[kidx].range_set = true;
    }
    return kidx;
  }

  bool wasProcessed(char *section, char *key) {
    int kidx = findKey(section, key);
    if (kidx >= 0 && keys[kidx].processed) return true;
    return false;
  }

  bool process(char *filename, char *section=NULL, bool use_global=false, bool verbose=true) {
     FILE *fp = fopen(filename, "r");
    if (fp == NULL) {
      if (verbose) std::cerr << "Error (Config::process): cannot open '" << filename << "'" << std::endl;
      return false;
    }
    char line[256], curr_section[80], *key, *value;
    curr_section[0] = '\0';
    int  kidx, count = 0, len, i, j;

    while (fgets(line, 255, fp)) {
      for (i = 0; line[i]; i++) if (line[i] == '#') { line[i] = '\0'; break; }//‰Šú‰»H
      len = i;
      // skip empty lines and commented lines
      if (line[0] == ' '  || line[0] == '\t' || line[0] == '\n') continue;
      // skip different sections
      if (line[0] == '[') {
	for (i = 0; line[i] && line[i]!=']'; i++); if (line[i]==']') line[i]='\0';
	for (i = i-1; line[i] == ' '; i--) line[i] = '\0';
	for (i = 1; line[i] == ' '; i++);
	strcpy( curr_section, line+i );
	continue;
      }
      while (line[len-1] == '\n' || line[len-1] == ' ' || line[len-1] == '\t') line[--len] = '\0';
      if (section == NULL || (section[0] == '\0' && curr_section[0] != '\0') ||
	  (section && strcmp(section, curr_section) != 0) ) continue;

      // read the key and value string
      for (i = 0; i < len; i++) if (line[i] == '=') { line[i] = '\0'; break; }
      if (i == len) continue;
      for (key   = line, j=i-1; key[j] == ' ' || key[j] == '\t'; j--) key[j] = '\0';
      for (value = line + i+1;  *value == ' ' || *value == '\t'; value++);
      if (value[0] == '\0') continue;  // value is not set, use default value
      if ((kidx = findKey( section, key, use_global )) < 0) continue;
      switch (keys[kidx].type) {
      case CFG_CHAR  : *((char*)  (keys[kidx].p)) = value[0];     break;
      case CFG_INT   : *((int*)   (keys[kidx].p)) = atoi(value);  break;
      case CFG_FLOAT : *((float*) (keys[kidx].p)) = (float)atof(value);  break;
      case CFG_DOUBLE: *((double*)(keys[kidx].p)) = atof(value);  break;
      case CFG_STRING: strcpy((char*)(keys[kidx].p), value);      break;
      case CFG_DIR   :
	len = (int)strlen(value);	// remove '/' at the end of the string
	if (len > 0 && value[len-1] == '/') value[len-1] = '\0';
	strcpy((char*)(keys[kidx].p), value);
	break;
      case CFG_BOOL  :
	if (value[0] == '\n' || value[0] == '\0' || value[0] == '0'  ||
	    ((value[0] == 'N' || value[0] == 'n') && value[1] == 'o') ||
	    ((value[0] == 'F' || value[0] == 'f') && value[1] == 'a' && value[2] == 'l'))
	  *((bool*)  (keys[kidx].p)) = false;
	else *((bool*)  (keys[kidx].p)) = true;
	break;
      }
      // check value range
      if (keys[kidx].range_set) {
	switch (keys[kidx].type) {
	case CFG_INT:
	  if (*((int*)(keys[kidx].p)) < (int)(keys[kidx].range[0])) *((int*)(keys[kidx].p)) = (int)(keys[kidx].range[0]);
	  if (*((int*)(keys[kidx].p)) > (int)(keys[kidx].range[1])) *((int*)(keys[kidx].p)) = (int)(keys[kidx].range[1]);
	  break;
	case CFG_FLOAT:
	  if (*((float*)(keys[kidx].p)) < keys[kidx].range[0]) *((float*)(keys[kidx].p)) = (float)keys[kidx].range[0];
	  if (*((float*)(keys[kidx].p)) > keys[kidx].range[1]) *((float*)(keys[kidx].p)) = (float)keys[kidx].range[1];
	  break;
	case CFG_DOUBLE:
	  if (*((double*)(keys[kidx].p)) < keys[kidx].range[0]) *((double*)(keys[kidx].p)) = keys[kidx].range[0];
	  if (*((double*)(keys[kidx].p)) > keys[kidx].range[1]) *((double*)(keys[kidx].p)) = keys[kidx].range[1];
	  break;
	case CFG_CHAR:
	  if (*((char*)(keys[kidx].p)) < (char)(keys[kidx].range[0])) *((char*)(keys[kidx].p)) = (char)(keys[kidx].range[0]);
	  if (*((char*)(keys[kidx].p)) > (char)(keys[kidx].range[1])) *((char*)(keys[kidx].p)) = (char)(keys[kidx].range[1]);
	  break;
	}
      }
      keys[kidx].processed = true;
      count++;
    }
    fclose(fp);
    return true;
  }

  void printInfo(char *title=NULL) {
    int  sidx, kidx, i;
    char buf[256];
    if (title) std::cout << title << std::endl;
    else std::cout << "GUIH::Config Information" << std::endl;
    for (sidx = 0; sidx < nsections; sidx++) {
      if (sidx > 0) std::cout << "  [" << sections[sidx] << "]" << std::endl;
      std::cout.setf(std::ios::left);
      for (kidx = 0; kidx < nkeys; kidx++) {
	if (keys[kidx].sidx != sidx) continue;
	std::cout << "  " << std::setw(CFG_NAME_MAX_LEN) << keys[kidx].key << " ";	// names
	switch (keys[kidx].type) {				// type
	case CFG_CHAR:   std::cout << std::setw(3) << "chr "; break;
	case CFG_INT:    std::cout << std::setw(3) << "int "; break;
	case CFG_FLOAT:  std::cout << std::setw(3) << "flt "; break;
	case CFG_DOUBLE: std::cout << std::setw(3) << "dbl "; break;
	case CFG_STRING: std::cout << std::setw(3) << "str "; break;
	case CFG_DIR:    std::cout << std::setw(3) << "dir "; break;
	case CFG_BOOL:   std::cout << std::setw(3) << "bln "; break;
	default:         std::cout << std::setw(3) << "    "; break;
	}
	std::cout << (keys[kidx].processed ? "[Y] " : "[ ] ");	// processed ?
	void *v = keys[kidx].p;
	buf[0] = '\0';
	switch (keys[kidx].type) {
	case CFG_CHAR:   sprintf(buf+strlen(buf), "%c", *((char*)   v));  break;
	case CFG_INT:    sprintf(buf+strlen(buf), "%d", *((int*)    v)); break;
	case CFG_FLOAT:  sprintf(buf+strlen(buf), "%g", (*((float*) v) == CFG_UNSET ? 0 : *((float*)(v)))); break;
	case CFG_DOUBLE: sprintf(buf+strlen(buf), "%g", (*((double*)v) == CFG_UNSET ? 0 : *((double*)(v)))); break;
	case CFG_STRING: sprintf(buf+strlen(buf), "\"%s\"", (char*)     v); break;
	case CFG_DIR:    sprintf(buf+strlen(buf), "\"%s\"", (char*)     v); break;
	case CFG_BOOL:   sprintf(buf+strlen(buf), "%s", (*((bool*)  v) ? "true" : "false")); break;
	default: buf[strlen(buf)] = '\0'; break;
	}
	i = (keys[kidx].type == CFG_STRING || keys[kidx].type == CFG_DIR ? 28 : 18);
	std::cout << std::setw(i) << buf << " ";		// value(s)
	if (keys[kidx].range_set) {				// Range[min~max]
	  double *r = keys[kidx].range;
	  std::cout << "Range[";
	  switch (keys[kidx].type) {
	  case CFG_CHAR:   std::cout << (char)  r[0] << " ~ " << (char)  r[1]; break;
	  case CFG_INT:    std::cout << (int)   r[0] << " ~ " << (int)   r[1]; break;
	  case CFG_FLOAT:  std::cout << (float) r[0] << " ~ " << (float) r[1]; break;
	  case CFG_DOUBLE: std::cout << (double)r[0] << " ~ " << (double)r[1]; break;
	  }
	  std::cout << "]";
	}
	std::cout << std::endl;
      }
    }
  }

  bool writeFile(char *filename, bool show_message=true) {
    FILE *fp = fopen(filename, "w+");
    if (fp == NULL) return false;
    fprintf(fp, "# Initialization/Configuration file\n");
    int sidx, kidx;
    for (sidx = 0; sidx < nsections; sidx++) {
      if (sidx > 0) fprintf(fp, "\n[%s]\n", sections[sidx]);
      else          fprintf(fp, "\n");
      for (kidx = 0; kidx < nkeys; kidx++) {
	if (keys[kidx].sidx != sidx) continue;
	void *v = keys[kidx].p;
	switch (keys[kidx].type) {
	case CFG_CHAR:   fprintf(fp, "%s\t= %c\n", keys[kidx].key, *((char*)  v)); break;
	case CFG_INT:    fprintf(fp, "%s\t= %d\n", keys[kidx].key, *((int*)   v)); break;
	case CFG_FLOAT:  fprintf(fp, "%s\t= %g\n", keys[kidx].key, *((float*) v)); break;
	case CFG_DOUBLE: fprintf(fp, "%s\t= %g\n", keys[kidx].key, *((double*)v)); break;
	case CFG_STRING: fprintf(fp, "%s\t= %s\n", keys[kidx].key,   (char*)  v ); break;
	case CFG_DIR:    fprintf(fp, "%s\t= %s\n", keys[kidx].key,   (char*)  v ); break;
	case CFG_BOOL:   fprintf(fp, "%s\t= %s\n", keys[kidx].key, (*((bool*)v) ? "true" : "false")); break;
	default: break;
	}
      }
    }
    fclose(fp);
    if (show_message) std::cout << "Configuration file created as '" << filename << "'" << std::endl;
    return true;
  }

private:
  int  findSection(char *section) {
    int i;
    if (section == NULL) return 0;
    for (i = 0; i < nsections; i++)
      if (strncmp(section, sections[i], CFG_NAME_MAX_LEN)==0) break;
    return (i < nsections ? i : -1);
  }

  int  findKey(char *section, char *key, bool use_global=true) {
    return findKey( findSection(section), key, use_global );
  }

  int  findKey(int sidx, char *key, bool use_global=true) {
    int i;
    // look for an exact match
    if (sidx >= 0) {
      for (i = 0; i < nkeys; i++)
	if (keys[i].sidx == sidx &&
	    strncmp(keys[i].key, key, CFG_NAME_MAX_LEN) == 0) break;
      if (i < nkeys) return i;
    }
    // if not found, look for the global key
    if (use_global) {
      for (i = 0; i < nkeys; i++)
	if (keys[i].sidx == 0 &&
	    strncmp(keys[i].key, key, CFG_NAME_MAX_LEN) == 0) break;
      if (i < nkeys) return i;
    }
    return -1;
  }

};


}	// end of GUIH namespace


#endif	// GUIH_CONFIG_HPP


// ===================================================================
#if 0	// Example code starts
// ===================================================================
// ---- "configurationA" section in the file "my_app.cfg" ----
// [ configurationA ]                    # spaces in "[configurationA]" do not matter
// Width = 320                           # this is comment
// Height = 240                          # this is comment
// Density=5.7                           # this is comment
// KeyStr=The rain falls in the plain.   # this is comment
// MultiV=1024, 768, 8                   # this is comment
// -----------------------------------------------------------
#include <iostream>
#include "guih_config.hpp"
using namespace std;
int main(int argc, char **argv)
{
  GUIH::Config cfg;
  int   cfgWidth, cfgHeight, v1, v2, v3;
  float cfgDensity;
  char  cfgKey[80], cfgMultiV[80];
  // The type can be either CFG_BOOL, CFG_CHAR, CFG_INT, CFG_FLOAT, CFG_DOUBLE, CFG_STRING, or CFG_DIR
  char *cfg_file_name    = "my_app.cfg";
  char *cfg_section_name = "configurationA";
  cfg.set(cfg_section_name, "Width", CFG_INT, &cfgWidth); // single value (it's set by default to 0)
  cfg.set(cfg_section_name, "Height", CFG_INT, &cfgHeight, 240); // single value with default
  cfg.set(cfg_section_name, "Density", CFG_FLOAT, &cfgDensity, 5.0, 1.0, 9.0); // single value with default and min/max range
  cfg.set(cfg_section_name, "KeyStr", CFG_STRING, cfgKey);       // string (it is set by default to "")
  cfg.set(cfg_section_name, "MultiV", CFG_STRING, cfgMultiV);    // string (it is set by default to "")
  if (!cfg.process(cfg_file_name, cfg_section_name)) exit(1);

  // Now use the values for whatever you want..
  printf( "'%s'\n", cfgKey);  // 'The rain falls in the plain.'
  sscanf( cfgMultiV, "%d, %d, %d", &v1, &v2, &v3 );
  // To tell whether it was actually given the default value or
  // it wasn't found at all in the configuration file, use wasProcessed().
  if (cfg.wasProcessed("Width")) printf("'Width' found in the config file\n");
  else                       printf("'Width' not found in the config file\n");

  return 0;
}
// ===================================================================
#endif	// Example code ends
// ===================================================================
