// Copyright 2024 RaptorLift
// Licensed under the Apache License, Version 2.0

#ifndef RAPTORLIFT_HARDWARE__VISIBILITY_CONTROL_H_
#define RAPTORLIFT_HARDWARE__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define RAPTORLIFT_HARDWARE_EXPORT __attribute__((dllexport))
#define RAPTORLIFT_HARDWARE_IMPORT __attribute__((dllimport))
#else
#define RAPTORLIFT_HARDWARE_EXPORT __declspec(dllexport)
#define RAPTORLIFT_HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef RAPTORLIFT_HARDWARE_BUILDING_DLL
#define RAPTORLIFT_HARDWARE_PUBLIC RAPTORLIFT_HARDWARE_EXPORT
#else
#define RAPTORLIFT_HARDWARE_PUBLIC RAPTORLIFT_HARDWARE_IMPORT
#endif
#define RAPTORLIFT_HARDWARE_PUBLIC_TYPE RAPTORLIFT_HARDWARE_PUBLIC
#define RAPTORLIFT_HARDWARE_LOCAL
#else
#define RAPTORLIFT_HARDWARE_EXPORT __attribute__((visibility("default")))
#define RAPTORLIFT_HARDWARE_IMPORT
#if __GNUC__ >= 4
#define RAPTORLIFT_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define RAPTORLIFT_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define RAPTORLIFT_HARDWARE_PUBLIC
#define RAPTORLIFT_HARDWARE_LOCAL
#endif
#define RAPTORLIFT_HARDWARE_PUBLIC_TYPE
#endif

#endif  // RAPTORLIFT_HARDWARE__VISIBILITY_CONTROL_H_
