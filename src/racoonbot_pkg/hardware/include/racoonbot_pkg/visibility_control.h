// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef RACOONBOT_PKG__VISIBILITY_CONTROL_H_
#define RACOONBOT_PKG__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define RACOONBOT_PKG_EXPORT __attribute__((dllexport))
#define RACOONBOT_PKG_IMPORT __attribute__((dllimport))
#else
#define RACOONBOT_PKG_EXPORT __declspec(dllexport)
#define RACOONBOT_PKG_IMPORT __declspec(dllimport)
#endif
#ifdef RACOONBOT_PKG_BUILDING_DLL
#define RACOONBOT_PKG_PUBLIC RACOONBOT_PKG_EXPORT
#else
#define RACOONBOT_PKG_PUBLIC RACOONBOT_PKG_IMPORT
#endif
#define RACOONBOT_PKG_PUBLIC_TYPE RACOONBOT_PKG_PUBLIC
#define RACOONBOT_PKG_LOCAL
#else
#define RACOONBOT_PKG_EXPORT __attribute__((visibility("default")))
#define RACOONBOT_PKG_IMPORT
#if __GNUC__ >= 4
#define RACOONBOT_PKG_PUBLIC __attribute__((visibility("default")))
#define RACOONBOT_PKG_LOCAL __attribute__((visibility("hidden")))
#else
#define RACOONBOT_PKG_PUBLIC
#define RACOONBOT_PKG_LOCAL
#endif
#define RACOONBOT_PKG_PUBLIC_TYPE
#endif

#endif  // RACOONBOT_PKG__VISIBILITY_CONTROL_H_
