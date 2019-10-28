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
enum NavMeshErrorCode
{
	NMSUCCESS = 0x0,	// No issues
	NMRECAST = 0x1,		// Caused by Recast
	NMDETOUR = 0x2,		// Caused by Detour
	NMINPUT = 0x4,		// Incorrect input
	NMALLOCATION = 0x8, // Allocation failed, most likely out of memory
	NMCREATION = 0x16,	// A R/D function failed to create a structure
	NMIO = 0x32			// Issues with I/O
};

//  +-----------------------------------------------------------------------------+
//  |  NavMeshError                                                               |
//  |  Handles errors, logging, and error code maintenance.                 LH2'19|
//  +-----------------------------------------------------------------------------+
static int NavMeshError(int code, const char* format, ...)
{
	va_list ap;
	__crt_va_start(ap, format);
	vprintf(format, ap);
	__crt_va_end(ap);
	printf("\n");

	return code;
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshError                                                               |
//  |  Handles errors, logging, and error code maintenance.                 LH2'19|
//  +-----------------------------------------------------------------------------+
static int NavMeshError(int* internalErrorCode, int code, const char* format, ...)
{
	if (code) *internalErrorCode = code;

	va_list ap;
	__crt_va_start(ap, format);
	vprintf(format, ap);
	__crt_va_end(ap);
	printf("\n");

	return code;
}

} // namespace Lighthouse2

// EOF