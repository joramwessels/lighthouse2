/* navmesh_common.h - Copyright 2019 Utrecht University

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

	   http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#pragma once

namespace lighthouse2 {

#define DETOUR_MAX_NAVMESH_NODES 2048

//  +-----------------------------------------------------------------------------+
//  |  NavMeshErrorCode                                                           |
//  |  Encodes bitwise info about the last error.                                 |
//  |  Error codes can be interpreted by bitwise & operations.              LH2'19|
//  +-----------------------------------------------------------------------------+
struct NavMeshStatus
{
	enum Code
	{
		SUCCESS = 0x0,	// No issues
		RC = 0x1,		// Caused by Recast
		DT = 0x2,		// Caused by Detour
		INPUT = 0x4,	// Incorrect input
		MEM = 0x8,		// Allocation failed, most likely out of memory
		INIT = 0x16,	// A R/D function failed to create a structure
		IO = 0x32		// Issues with I/O
	};
	int code;

	NavMeshStatus() { code = SUCCESS; };
	NavMeshStatus(const int& a) { code = a; };
	NavMeshStatus& operator=(const int& a) { code = a; return *this; };
	operator int() const { return code; };

	bool Success() const { return (code == SUCCESS); };
	bool Failed() const { return (code != SUCCESS); };
};

//  +-----------------------------------------------------------------------------+
//  |  NavMeshStatus                                                              |
//  |  Updates the error code of the class, and prints the message to stdout.     |
//  |																			  |
//  |  *internalStatus* is a ptr to the error status of the class, in order       |
//  |  to automatically update the internal status. Use 0 if not needed.          |
//  |  *code* is the NavMeshErrorCode of the error. NMSUCCESS for logging.        |
//  |  *prefix* is an optional string printed before the error message.           |
//  |  *format* and the variadic arguments after that describe the error.         |
//  |																			  |
//  |  The return value is the error code, for chaining purposes.           LH2'19|
//  +-----------------------------------------------------------------------------+
static NavMeshStatus NavMeshError(NavMeshStatus* internalStatus, NavMeshStatus code, const char* prefix, const char* format, ...)
{
	if (internalStatus && code.Failed()) *internalStatus = code;
	printf(prefix);

	va_list ap;
	__crt_va_start(ap, format);
	vprintf(format, ap);
	__crt_va_end(ap);

	return code;
}

} // namespace Lighthouse2

// EOF