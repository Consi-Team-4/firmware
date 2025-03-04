#include "debug.h"

// #include "error.h"
// PICO_OK = 0,                                ///< No error; the operation succeeded
// PICO_ERROR_NONE = 0,                        ///< No error; the operation succeeded
// PICO_ERROR_GENERIC = -1,                    ///< An unspecified error occurred
// PICO_ERROR_TIMEOUT = -2,                    ///< The function failed due to timeout
// PICO_ERROR_NO_DATA = -3,                    ///< Attempt for example to read from an empty buffer/FIFO
// PICO_ERROR_NOT_PERMITTED = -4,              ///< Permission violation e.g. write to read-only flash partition, or security violation
// PICO_ERROR_INVALID_ARG = -5,                ///< Argument is outside of range of supported values`
// PICO_ERROR_IO = -6,                         ///< An I/O error occurred
// PICO_ERROR_BADAUTH = -7,                    ///< The authorization failed due to bad credentials
// PICO_ERROR_CONNECT_FAILED = -8,             ///< The connection failed
// PICO_ERROR_INSUFFICIENT_RESOURCES = -9,     ///< Dynamic allocation of resources failed
// PICO_ERROR_INVALID_ADDRESS = -10,           ///< Address argument was out-of-bounds or was determined to be an address that the caller may not access
// PICO_ERROR_BAD_ALIGNMENT = -11,             ///< Address was mis-aligned (usually not on word boundary)
// PICO_ERROR_INVALID_STATE = -12,             ///< Something happened or failed to happen in the past, and consequently we (currently) can't service the request
// PICO_ERROR_BUFFER_TOO_SMALL = -13,          ///< A user-allocated buffer was too small to hold the result or working state of this function
// PICO_ERROR_PRECONDITION_NOT_MET = -14,      ///< The call failed because another function must be called first
// PICO_ERROR_MODIFIED_DATA = -15,             ///< Cached data was determined to be inconsistent with the actual version of the data
// PICO_ERROR_INVALID_DATA = -16,              ///< A data structure failed to validate
// PICO_ERROR_NOT_FOUND = -17,                 ///< Attempted to access something that does not exist; or, a search failed
// PICO_ERROR_UNSUPPORTED_MODIFICATION = -18,  ///< Write is impossible based on previous writes; e.g. attempted to clear an OTP bit
// PICO_ERROR_LOCK_REQUIRED = -19,             ///< A required lock is not owned
// PICO_ERROR_VERSION_MISMATCH = -20,          ///< A version mismatch occurred (e.g. trying to run PIO version 1 code on RP2040)
// PICO_ERROR_RESOURCE_IN_USE = -21            ///< The call could not proceed because requires resourcesw were unavailable

const char* errorToString(int errorCode) {
    const char* table[22] = {
        "PICO_OK",
        "PICO_ERROR_GENERIC",
        "PICO_ERROR_TIMEOUT",
        "PICO_ERROR_NO_DATA",
        "PICO_ERROR_NOT_PERMITTED",
        "PICO_ERROR_INVALID_ARG",
        "PICO_ERROR_IO",
        "PICO_ERROR_BADAUTH",
        "PICO_ERROR_CONNECT_FAILED",
        "PICO_ERROR_INSUFFICIENT_RESOURCES",
        "PICO_ERROR_INVALID_ADDRESS",
        "PICO_ERROR_BAD_ALIGNMENT",
        "PICO_ERROR_INVALID_STATE",
        "PICO_ERROR_BUFFER_TOO_SMALL",
        "PICO_ERROR_PRECONDITION_NOT_MET",
        "PICO_ERROR_MODIFIED_DATA",
        "PICO_ERROR_INVALID_DATA",
        "PICO_ERROR_NOT_FOUND",
        "PICO_ERROR_UNSUPPORTED_MODIFICATION",
        "PICO_ERROR_LOCK_REQUIRED",
        "PICO_ERROR_VERSION_MISMATCH",
        "PICO_ERROR_RESOURCE_IN_USE",
    };

    if (errorCode > 0 || errorCode < -21) {
        return "UNKNOWN_ERROR";
    } else {
        return table[-errorCode];
    }
}
