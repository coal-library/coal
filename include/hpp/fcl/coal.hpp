#ifndef HPP_FCL_COAL_HPP
#define HPP_FCL_COAL_HPP

#include <coal/config.hh>
#include <coal/deprecated.hh>

#define COAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL

// Don't setup warning because there can be ignored by -isystem
#ifndef COAL_DISABLE_HPP_FCL_WARNINGS
#pragma message( \
    "Please update your includes from 'hpp/fcl' to 'coal' or define COAL_DISABLE_HPP_FCL_WARNINGS")
#endif

#define HPP_FCL_VERSION_AT_LEAST(major, minor, patch) \
  COAL_VERSION_AT_LEAST(major, minor, patch)

#endif  // COAL_FWD_HPP
