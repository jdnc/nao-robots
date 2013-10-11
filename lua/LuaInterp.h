#ifndef _LUA_INTERP_H
#define _LUA_INTERP_H

#define lua_c

extern "C" {
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
}

// This is the actual lua interpreter
class LuaInterp
{
 public:
  static lua_State *globalL;
  static const char *progname;

  lua_State *L;

 public:

  LuaInterp();
  ~LuaInterp();

  bool require( const char* lib );
  bool call( const char* cmd );
  void startInterpreter();

 public:
  /* C functions taken from lua.c */
  static void l_message (const char *pname, const char *msg);
  static int report (lua_State *L, int status);
  static int traceback (lua_State *L);
  static int docall (lua_State *L, int narg, int clear);
  static void print_version ();
  static int dofile (lua_State *L, const char *name);
  static int dostring (lua_State *L, const char *s, const char *name);
  static int dolibrary (lua_State *L, const char *name);
  static const char* get_prompt (lua_State *L, int firstline);
  static int incomplete (lua_State *L, int status);
  static int pushline (lua_State *L, int firstline);
  static int loadline (lua_State *L);
  
} ;

#endif // _LUA_H
