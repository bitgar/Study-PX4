/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/bitgar/PX4-Autopilot/src/drivers/uavcan/libuavcan/dsdl/uavcan/equipment/gnss/1063.Fix2.uavcan
 */

#ifndef UAVCAN_EQUIPMENT_GNSS_FIX2_HPP_INCLUDED
#define UAVCAN_EQUIPMENT_GNSS_FIX2_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

#include <uavcan/Timestamp.hpp>
#include <uavcan/equipment/gnss/ECEFPositionVelocity.hpp>

/******************************* Source text **********************************
#
# GNSS ECEF and LLA navigation solution with uncertainty.
#

#
# Global network-synchronized time, if available, otherwise zero.
#
uavcan.Timestamp timestamp

#
# Time solution.
# The method and number of leap seconds which were in use for deriving the timestamp are
# defined in the fields below.
#
uavcan.Timestamp gnss_timestamp

#
# Method used for deriving the GNSS timestamp field.
# This data type relies on the following definitions:
#
#   Leap seconds  - Accumulated one-second adjustments applied to UTC since 1972.
#                   For reference, on May 2017, the number of leap seconds was equal 27.
#                   The number of leap seconds is taken from the field num_leap_seconds.
#                   Refer to https://en.wikipedia.org/wiki/Leap_second for a general overview.
#
#   TAI timestamp - The number of microseconds between the current TAI time and
#                   the TAI time at UTC 1970-01-01T00:00:00.
#
#   UTC timestamp - The number of microseconds between the current UTC time and
#                   UTC 1970-01-01T00:00:00.
#                   UTC can be expressed via TAI as follows (in seconds):
#                       UTC = TAI - num_leap_seconds - 10
#                   And via GPS (in seconds):
#                       UTC = GPS - num_leap_seconds + 9
#
#   GPS timestamp - The number of microseconds between the current GPS time and
#                   the GPS time at UTC 1970-01-01T00:00:00.
#                   GPS time can be expressed via TAI as follows (in seconds):
#                       GPS = TAI - 19
#
uint3 GNSS_TIME_STANDARD_NONE = 0  # Time is unknown
uint3 GNSS_TIME_STANDARD_TAI  = 1
uint3 GNSS_TIME_STANDARD_UTC  = 2
uint3 GNSS_TIME_STANDARD_GPS  = 3
uint3 gnss_time_standard

void13   # Reserved space

#
# Accumulated one-second adjustments applied to UTC since 1972.
# The number must agree with the currently correct number of UTC leap seconds. If this cannot
# be garanteed, the field must be set to NUM_LEAP_SECONDS_UNKNOWN.
#
uint8 NUM_LEAP_SECONDS_UNKNOWN = 0
uint8 num_leap_seconds

#
# Position and velocity solution
#
int37 longitude_deg_1e8            # Longitude degrees multiplied by 1e8 (approx. 1 mm per LSB)
int37 latitude_deg_1e8             # Latitude degrees multiplied by 1e8 (approx. 1 mm per LSB on equator)
int27 height_ellipsoid_mm          # Height above ellipsoid in millimeters
int27 height_msl_mm                # Height above mean sea level in millimeters

float32[3] ned_velocity            # NED frame (north-east-down) in meters per second

#
# Fix status
#
uint6 sats_used

uint2 STATUS_NO_FIX    = 0
uint2 STATUS_TIME_ONLY = 1
uint2 STATUS_2D_FIX    = 2
uint2 STATUS_3D_FIX    = 3
uint2 status

#
# GNSS Mode
#
uint4 MODE_SINGLE      = 0
uint4 MODE_DGPS        = 1
uint4 MODE_RTK         = 2
uint4 MODE_PPP         = 3
uint4 mode

#
# GNSS Sub mode
#
uint6 SUB_MODE_DGPS_OTHER    = 0
uint6 SUB_MODE_DGPS_SBAS     = 1

uint6 SUB_MODE_RTK_FLOAT     = 0
uint6 SUB_MODE_RTK_FIXED     = 1

uint6 sub_mode

#
# Precision
#
float16[<=36] covariance    # Position and velocity covariance. Units are
                            # m^2 for position, (m/s)^2 for velocity and
                            # m^2/s for position/velocity.

float16 pdop

#
# Position and velocity solution in ECEF, if available
#
ECEFPositionVelocity[<=1] ecef_position_velocity
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.equipment.gnss.Fix2
uavcan.Timestamp timestamp
uavcan.Timestamp gnss_timestamp
saturated uint3 gnss_time_standard
void13
saturated uint8 num_leap_seconds
saturated int37 longitude_deg_1e8
saturated int37 latitude_deg_1e8
saturated int27 height_ellipsoid_mm
saturated int27 height_msl_mm
saturated float32[3] ned_velocity
saturated uint6 sats_used
saturated uint2 status
saturated uint4 mode
saturated uint6 sub_mode
saturated float16[<=36] covariance
saturated float16 pdop
uavcan.equipment.gnss.ECEFPositionVelocity[<=1] ecef_position_velocity
******************************************************************************/

#undef timestamp
#undef gnss_timestamp
#undef gnss_time_standard
#undef _void_0
#undef num_leap_seconds
#undef longitude_deg_1e8
#undef latitude_deg_1e8
#undef height_ellipsoid_mm
#undef height_msl_mm
#undef ned_velocity
#undef sats_used
#undef status
#undef mode
#undef sub_mode
#undef covariance
#undef pdop
#undef ecef_position_velocity
#undef GNSS_TIME_STANDARD_NONE
#undef GNSS_TIME_STANDARD_TAI
#undef GNSS_TIME_STANDARD_UTC
#undef GNSS_TIME_STANDARD_GPS
#undef NUM_LEAP_SECONDS_UNKNOWN
#undef STATUS_NO_FIX
#undef STATUS_TIME_ONLY
#undef STATUS_2D_FIX
#undef STATUS_3D_FIX
#undef MODE_SINGLE
#undef MODE_DGPS
#undef MODE_RTK
#undef MODE_PPP
#undef SUB_MODE_DGPS_OTHER
#undef SUB_MODE_DGPS_SBAS
#undef SUB_MODE_RTK_FLOAT
#undef SUB_MODE_RTK_FIXED

namespace uavcan
{
namespace equipment
{
namespace gnss
{

template <int _tmpl>
struct UAVCAN_EXPORT Fix2_
{
    typedef const Fix2_<_tmpl>& ParameterType;
    typedef Fix2_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
        typedef ::uavcan::IntegerSpec< 3, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > GNSS_TIME_STANDARD_NONE;
        typedef ::uavcan::IntegerSpec< 3, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > GNSS_TIME_STANDARD_TAI;
        typedef ::uavcan::IntegerSpec< 3, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > GNSS_TIME_STANDARD_UTC;
        typedef ::uavcan::IntegerSpec< 3, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > GNSS_TIME_STANDARD_GPS;
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > NUM_LEAP_SECONDS_UNKNOWN;
        typedef ::uavcan::IntegerSpec< 2, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > STATUS_NO_FIX;
        typedef ::uavcan::IntegerSpec< 2, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > STATUS_TIME_ONLY;
        typedef ::uavcan::IntegerSpec< 2, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > STATUS_2D_FIX;
        typedef ::uavcan::IntegerSpec< 2, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > STATUS_3D_FIX;
        typedef ::uavcan::IntegerSpec< 4, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > MODE_SINGLE;
        typedef ::uavcan::IntegerSpec< 4, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > MODE_DGPS;
        typedef ::uavcan::IntegerSpec< 4, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > MODE_RTK;
        typedef ::uavcan::IntegerSpec< 4, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > MODE_PPP;
        typedef ::uavcan::IntegerSpec< 6, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > SUB_MODE_DGPS_OTHER;
        typedef ::uavcan::IntegerSpec< 6, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > SUB_MODE_DGPS_SBAS;
        typedef ::uavcan::IntegerSpec< 6, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > SUB_MODE_RTK_FLOAT;
        typedef ::uavcan::IntegerSpec< 6, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > SUB_MODE_RTK_FIXED;
    };

    struct FieldTypes
    {
        typedef ::uavcan::Timestamp timestamp;
        typedef ::uavcan::Timestamp gnss_timestamp;
        typedef ::uavcan::IntegerSpec< 3, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > gnss_time_standard;
        typedef ::uavcan::IntegerSpec< 13, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > _void_0;
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > num_leap_seconds;
        typedef ::uavcan::IntegerSpec< 37, ::uavcan::SignednessSigned, ::uavcan::CastModeSaturate > longitude_deg_1e8;
        typedef ::uavcan::IntegerSpec< 37, ::uavcan::SignednessSigned, ::uavcan::CastModeSaturate > latitude_deg_1e8;
        typedef ::uavcan::IntegerSpec< 27, ::uavcan::SignednessSigned, ::uavcan::CastModeSaturate > height_ellipsoid_mm;
        typedef ::uavcan::IntegerSpec< 27, ::uavcan::SignednessSigned, ::uavcan::CastModeSaturate > height_msl_mm;
        typedef ::uavcan::Array< ::uavcan::FloatSpec< 32, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeStatic, 3 > ned_velocity;
        typedef ::uavcan::IntegerSpec< 6, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > sats_used;
        typedef ::uavcan::IntegerSpec< 2, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > status;
        typedef ::uavcan::IntegerSpec< 4, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > mode;
        typedef ::uavcan::IntegerSpec< 6, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > sub_mode;
        typedef ::uavcan::Array< ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeDynamic, 36 > covariance;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > pdop;
        typedef ::uavcan::Array< ::uavcan::equipment::gnss::ECEFPositionVelocity, ::uavcan::ArrayModeDynamic, 1 > ecef_position_velocity;
    };

    enum
    {
        MinBitLen
            = FieldTypes::timestamp::MinBitLen
            + FieldTypes::gnss_timestamp::MinBitLen
            + FieldTypes::gnss_time_standard::MinBitLen
            + FieldTypes::_void_0::MinBitLen
            + FieldTypes::num_leap_seconds::MinBitLen
            + FieldTypes::longitude_deg_1e8::MinBitLen
            + FieldTypes::latitude_deg_1e8::MinBitLen
            + FieldTypes::height_ellipsoid_mm::MinBitLen
            + FieldTypes::height_msl_mm::MinBitLen
            + FieldTypes::ned_velocity::MinBitLen
            + FieldTypes::sats_used::MinBitLen
            + FieldTypes::status::MinBitLen
            + FieldTypes::mode::MinBitLen
            + FieldTypes::sub_mode::MinBitLen
            + FieldTypes::covariance::MinBitLen
            + FieldTypes::pdop::MinBitLen
            + FieldTypes::ecef_position_velocity::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::timestamp::MaxBitLen
            + FieldTypes::gnss_timestamp::MaxBitLen
            + FieldTypes::gnss_time_standard::MaxBitLen
            + FieldTypes::_void_0::MaxBitLen
            + FieldTypes::num_leap_seconds::MaxBitLen
            + FieldTypes::longitude_deg_1e8::MaxBitLen
            + FieldTypes::latitude_deg_1e8::MaxBitLen
            + FieldTypes::height_ellipsoid_mm::MaxBitLen
            + FieldTypes::height_msl_mm::MaxBitLen
            + FieldTypes::ned_velocity::MaxBitLen
            + FieldTypes::sats_used::MaxBitLen
            + FieldTypes::status::MaxBitLen
            + FieldTypes::mode::MaxBitLen
            + FieldTypes::sub_mode::MaxBitLen
            + FieldTypes::covariance::MaxBitLen
            + FieldTypes::pdop::MaxBitLen
            + FieldTypes::ecef_position_velocity::MaxBitLen
    };

    // Constants
    static const typename ::uavcan::StorageType< typename ConstantTypes::GNSS_TIME_STANDARD_NONE >::Type GNSS_TIME_STANDARD_NONE; // 0
    static const typename ::uavcan::StorageType< typename ConstantTypes::GNSS_TIME_STANDARD_TAI >::Type GNSS_TIME_STANDARD_TAI; // 1
    static const typename ::uavcan::StorageType< typename ConstantTypes::GNSS_TIME_STANDARD_UTC >::Type GNSS_TIME_STANDARD_UTC; // 2
    static const typename ::uavcan::StorageType< typename ConstantTypes::GNSS_TIME_STANDARD_GPS >::Type GNSS_TIME_STANDARD_GPS; // 3
    static const typename ::uavcan::StorageType< typename ConstantTypes::NUM_LEAP_SECONDS_UNKNOWN >::Type NUM_LEAP_SECONDS_UNKNOWN; // 0
    static const typename ::uavcan::StorageType< typename ConstantTypes::STATUS_NO_FIX >::Type STATUS_NO_FIX; // 0
    static const typename ::uavcan::StorageType< typename ConstantTypes::STATUS_TIME_ONLY >::Type STATUS_TIME_ONLY; // 1
    static const typename ::uavcan::StorageType< typename ConstantTypes::STATUS_2D_FIX >::Type STATUS_2D_FIX; // 2
    static const typename ::uavcan::StorageType< typename ConstantTypes::STATUS_3D_FIX >::Type STATUS_3D_FIX; // 3
    static const typename ::uavcan::StorageType< typename ConstantTypes::MODE_SINGLE >::Type MODE_SINGLE; // 0
    static const typename ::uavcan::StorageType< typename ConstantTypes::MODE_DGPS >::Type MODE_DGPS; // 1
    static const typename ::uavcan::StorageType< typename ConstantTypes::MODE_RTK >::Type MODE_RTK; // 2
    static const typename ::uavcan::StorageType< typename ConstantTypes::MODE_PPP >::Type MODE_PPP; // 3
    static const typename ::uavcan::StorageType< typename ConstantTypes::SUB_MODE_DGPS_OTHER >::Type SUB_MODE_DGPS_OTHER; // 0
    static const typename ::uavcan::StorageType< typename ConstantTypes::SUB_MODE_DGPS_SBAS >::Type SUB_MODE_DGPS_SBAS; // 1
    static const typename ::uavcan::StorageType< typename ConstantTypes::SUB_MODE_RTK_FLOAT >::Type SUB_MODE_RTK_FLOAT; // 0
    static const typename ::uavcan::StorageType< typename ConstantTypes::SUB_MODE_RTK_FIXED >::Type SUB_MODE_RTK_FIXED; // 1

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::timestamp >::Type timestamp;
    typename ::uavcan::StorageType< typename FieldTypes::gnss_timestamp >::Type gnss_timestamp;
    typename ::uavcan::StorageType< typename FieldTypes::gnss_time_standard >::Type gnss_time_standard;
    typename ::uavcan::StorageType< typename FieldTypes::num_leap_seconds >::Type num_leap_seconds;
    typename ::uavcan::StorageType< typename FieldTypes::longitude_deg_1e8 >::Type longitude_deg_1e8;
    typename ::uavcan::StorageType< typename FieldTypes::latitude_deg_1e8 >::Type latitude_deg_1e8;
    typename ::uavcan::StorageType< typename FieldTypes::height_ellipsoid_mm >::Type height_ellipsoid_mm;
    typename ::uavcan::StorageType< typename FieldTypes::height_msl_mm >::Type height_msl_mm;
    typename ::uavcan::StorageType< typename FieldTypes::ned_velocity >::Type ned_velocity;
    typename ::uavcan::StorageType< typename FieldTypes::sats_used >::Type sats_used;
    typename ::uavcan::StorageType< typename FieldTypes::status >::Type status;
    typename ::uavcan::StorageType< typename FieldTypes::mode >::Type mode;
    typename ::uavcan::StorageType< typename FieldTypes::sub_mode >::Type sub_mode;
    typename ::uavcan::StorageType< typename FieldTypes::covariance >::Type covariance;
    typename ::uavcan::StorageType< typename FieldTypes::pdop >::Type pdop;
    typename ::uavcan::StorageType< typename FieldTypes::ecef_position_velocity >::Type ecef_position_velocity;

    Fix2_()
        : timestamp()
        , gnss_timestamp()
        , gnss_time_standard()
        , num_leap_seconds()
        , longitude_deg_1e8()
        , latitude_deg_1e8()
        , height_ellipsoid_mm()
        , height_msl_mm()
        , ned_velocity()
        , sats_used()
        , status()
        , mode()
        , sub_mode()
        , covariance()
        , pdop()
        , ecef_position_velocity()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<1769 == MaxBitLen>::check();
#endif
    }

    bool operator==(ParameterType rhs) const;
    bool operator!=(ParameterType rhs) const { return !operator==(rhs); }

    /**
     * This comparison is based on @ref uavcan::areClose(), which ensures proper comparison of
     * floating point fields at any depth.
     */
    bool isClose(ParameterType rhs) const;

    static int encode(ParameterType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    static int decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    /*
     * Static type info
     */
    enum { DataTypeKind = ::uavcan::DataTypeKindMessage };
    enum { DefaultDataTypeID = 1063 };

    static const char* getDataTypeFullName()
    {
        return "uavcan.equipment.gnss.Fix2";
    }

    static void extendDataTypeSignature(::uavcan::DataTypeSignature& signature)
    {
        signature.extend(getDataTypeSignature());
    }

    static ::uavcan::DataTypeSignature getDataTypeSignature();

};

/*
 * Out of line struct method definitions
 */

template <int _tmpl>
bool Fix2_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        timestamp == rhs.timestamp &&
        gnss_timestamp == rhs.gnss_timestamp &&
        gnss_time_standard == rhs.gnss_time_standard &&
        num_leap_seconds == rhs.num_leap_seconds &&
        longitude_deg_1e8 == rhs.longitude_deg_1e8 &&
        latitude_deg_1e8 == rhs.latitude_deg_1e8 &&
        height_ellipsoid_mm == rhs.height_ellipsoid_mm &&
        height_msl_mm == rhs.height_msl_mm &&
        ned_velocity == rhs.ned_velocity &&
        sats_used == rhs.sats_used &&
        status == rhs.status &&
        mode == rhs.mode &&
        sub_mode == rhs.sub_mode &&
        covariance == rhs.covariance &&
        pdop == rhs.pdop &&
        ecef_position_velocity == rhs.ecef_position_velocity;
}

template <int _tmpl>
bool Fix2_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(timestamp, rhs.timestamp) &&
        ::uavcan::areClose(gnss_timestamp, rhs.gnss_timestamp) &&
        ::uavcan::areClose(gnss_time_standard, rhs.gnss_time_standard) &&
        ::uavcan::areClose(num_leap_seconds, rhs.num_leap_seconds) &&
        ::uavcan::areClose(longitude_deg_1e8, rhs.longitude_deg_1e8) &&
        ::uavcan::areClose(latitude_deg_1e8, rhs.latitude_deg_1e8) &&
        ::uavcan::areClose(height_ellipsoid_mm, rhs.height_ellipsoid_mm) &&
        ::uavcan::areClose(height_msl_mm, rhs.height_msl_mm) &&
        ::uavcan::areClose(ned_velocity, rhs.ned_velocity) &&
        ::uavcan::areClose(sats_used, rhs.sats_used) &&
        ::uavcan::areClose(status, rhs.status) &&
        ::uavcan::areClose(mode, rhs.mode) &&
        ::uavcan::areClose(sub_mode, rhs.sub_mode) &&
        ::uavcan::areClose(covariance, rhs.covariance) &&
        ::uavcan::areClose(pdop, rhs.pdop) &&
        ::uavcan::areClose(ecef_position_velocity, rhs.ecef_position_velocity);
}

template <int _tmpl>
int Fix2_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    typename ::uavcan::StorageType< typename FieldTypes::_void_0 >::Type _void_0 = 0;
    int res = 1;
    res = FieldTypes::timestamp::encode(self.timestamp, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::gnss_timestamp::encode(self.gnss_timestamp, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::gnss_time_standard::encode(self.gnss_time_standard, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::_void_0::encode(_void_0, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::num_leap_seconds::encode(self.num_leap_seconds, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::longitude_deg_1e8::encode(self.longitude_deg_1e8, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::latitude_deg_1e8::encode(self.latitude_deg_1e8, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::height_ellipsoid_mm::encode(self.height_ellipsoid_mm, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::height_msl_mm::encode(self.height_msl_mm, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::ned_velocity::encode(self.ned_velocity, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::sats_used::encode(self.sats_used, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::status::encode(self.status, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::mode::encode(self.mode, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::sub_mode::encode(self.sub_mode, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::covariance::encode(self.covariance, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::pdop::encode(self.pdop, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::ecef_position_velocity::encode(self.ecef_position_velocity, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int Fix2_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    typename ::uavcan::StorageType< typename FieldTypes::_void_0 >::Type _void_0 = 0;
    int res = 1;
    res = FieldTypes::timestamp::decode(self.timestamp, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::gnss_timestamp::decode(self.gnss_timestamp, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::gnss_time_standard::decode(self.gnss_time_standard, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::_void_0::decode(_void_0, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::num_leap_seconds::decode(self.num_leap_seconds, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::longitude_deg_1e8::decode(self.longitude_deg_1e8, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::latitude_deg_1e8::decode(self.latitude_deg_1e8, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::height_ellipsoid_mm::decode(self.height_ellipsoid_mm, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::height_msl_mm::decode(self.height_msl_mm, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::ned_velocity::decode(self.ned_velocity, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::sats_used::decode(self.sats_used, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::status::decode(self.status, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::mode::decode(self.mode, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::sub_mode::decode(self.sub_mode, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::covariance::decode(self.covariance, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::pdop::decode(self.pdop, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::ecef_position_velocity::decode(self.ecef_position_velocity, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature Fix2_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x1404F437248B3AA9ULL);

    FieldTypes::timestamp::extendDataTypeSignature(signature);
    FieldTypes::gnss_timestamp::extendDataTypeSignature(signature);
    FieldTypes::gnss_time_standard::extendDataTypeSignature(signature);
    FieldTypes::_void_0::extendDataTypeSignature(signature);
    FieldTypes::num_leap_seconds::extendDataTypeSignature(signature);
    FieldTypes::longitude_deg_1e8::extendDataTypeSignature(signature);
    FieldTypes::latitude_deg_1e8::extendDataTypeSignature(signature);
    FieldTypes::height_ellipsoid_mm::extendDataTypeSignature(signature);
    FieldTypes::height_msl_mm::extendDataTypeSignature(signature);
    FieldTypes::ned_velocity::extendDataTypeSignature(signature);
    FieldTypes::sats_used::extendDataTypeSignature(signature);
    FieldTypes::status::extendDataTypeSignature(signature);
    FieldTypes::mode::extendDataTypeSignature(signature);
    FieldTypes::sub_mode::extendDataTypeSignature(signature);
    FieldTypes::covariance::extendDataTypeSignature(signature);
    FieldTypes::pdop::extendDataTypeSignature(signature);
    FieldTypes::ecef_position_velocity::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

template <int _tmpl>
const typename ::uavcan::StorageType< typename Fix2_<_tmpl>::ConstantTypes::GNSS_TIME_STANDARD_NONE >::Type
    Fix2_<_tmpl>::GNSS_TIME_STANDARD_NONE = 0U; // 0

template <int _tmpl>
const typename ::uavcan::StorageType< typename Fix2_<_tmpl>::ConstantTypes::GNSS_TIME_STANDARD_TAI >::Type
    Fix2_<_tmpl>::GNSS_TIME_STANDARD_TAI = 1U; // 1

template <int _tmpl>
const typename ::uavcan::StorageType< typename Fix2_<_tmpl>::ConstantTypes::GNSS_TIME_STANDARD_UTC >::Type
    Fix2_<_tmpl>::GNSS_TIME_STANDARD_UTC = 2U; // 2

template <int _tmpl>
const typename ::uavcan::StorageType< typename Fix2_<_tmpl>::ConstantTypes::GNSS_TIME_STANDARD_GPS >::Type
    Fix2_<_tmpl>::GNSS_TIME_STANDARD_GPS = 3U; // 3

template <int _tmpl>
const typename ::uavcan::StorageType< typename Fix2_<_tmpl>::ConstantTypes::NUM_LEAP_SECONDS_UNKNOWN >::Type
    Fix2_<_tmpl>::NUM_LEAP_SECONDS_UNKNOWN = 0U; // 0

template <int _tmpl>
const typename ::uavcan::StorageType< typename Fix2_<_tmpl>::ConstantTypes::STATUS_NO_FIX >::Type
    Fix2_<_tmpl>::STATUS_NO_FIX = 0U; // 0

template <int _tmpl>
const typename ::uavcan::StorageType< typename Fix2_<_tmpl>::ConstantTypes::STATUS_TIME_ONLY >::Type
    Fix2_<_tmpl>::STATUS_TIME_ONLY = 1U; // 1

template <int _tmpl>
const typename ::uavcan::StorageType< typename Fix2_<_tmpl>::ConstantTypes::STATUS_2D_FIX >::Type
    Fix2_<_tmpl>::STATUS_2D_FIX = 2U; // 2

template <int _tmpl>
const typename ::uavcan::StorageType< typename Fix2_<_tmpl>::ConstantTypes::STATUS_3D_FIX >::Type
    Fix2_<_tmpl>::STATUS_3D_FIX = 3U; // 3

template <int _tmpl>
const typename ::uavcan::StorageType< typename Fix2_<_tmpl>::ConstantTypes::MODE_SINGLE >::Type
    Fix2_<_tmpl>::MODE_SINGLE = 0U; // 0

template <int _tmpl>
const typename ::uavcan::StorageType< typename Fix2_<_tmpl>::ConstantTypes::MODE_DGPS >::Type
    Fix2_<_tmpl>::MODE_DGPS = 1U; // 1

template <int _tmpl>
const typename ::uavcan::StorageType< typename Fix2_<_tmpl>::ConstantTypes::MODE_RTK >::Type
    Fix2_<_tmpl>::MODE_RTK = 2U; // 2

template <int _tmpl>
const typename ::uavcan::StorageType< typename Fix2_<_tmpl>::ConstantTypes::MODE_PPP >::Type
    Fix2_<_tmpl>::MODE_PPP = 3U; // 3

template <int _tmpl>
const typename ::uavcan::StorageType< typename Fix2_<_tmpl>::ConstantTypes::SUB_MODE_DGPS_OTHER >::Type
    Fix2_<_tmpl>::SUB_MODE_DGPS_OTHER = 0U; // 0

template <int _tmpl>
const typename ::uavcan::StorageType< typename Fix2_<_tmpl>::ConstantTypes::SUB_MODE_DGPS_SBAS >::Type
    Fix2_<_tmpl>::SUB_MODE_DGPS_SBAS = 1U; // 1

template <int _tmpl>
const typename ::uavcan::StorageType< typename Fix2_<_tmpl>::ConstantTypes::SUB_MODE_RTK_FLOAT >::Type
    Fix2_<_tmpl>::SUB_MODE_RTK_FLOAT = 0U; // 0

template <int _tmpl>
const typename ::uavcan::StorageType< typename Fix2_<_tmpl>::ConstantTypes::SUB_MODE_RTK_FIXED >::Type
    Fix2_<_tmpl>::SUB_MODE_RTK_FIXED = 1U; // 1

/*
 * Final typedef
 */
typedef Fix2_<0> Fix2;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::uavcan::equipment::gnss::Fix2 > _uavcan_gdtr_registrator_Fix2;

}

} // Namespace gnss
} // Namespace equipment
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::equipment::gnss::Fix2 >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::equipment::gnss::Fix2::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::equipment::gnss::Fix2 >::stream(Stream& s, ::uavcan::equipment::gnss::Fix2::ParameterType obj, const int level)
{
    (void)s;
    (void)obj;
    (void)level;
    if (level > 0)
    {
        s << '\n';
        for (int pos = 0; pos < level; pos++)
        {
            s << "  ";
        }
    }
    s << "timestamp: ";
    YamlStreamer< ::uavcan::equipment::gnss::Fix2::FieldTypes::timestamp >::stream(s, obj.timestamp, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "gnss_timestamp: ";
    YamlStreamer< ::uavcan::equipment::gnss::Fix2::FieldTypes::gnss_timestamp >::stream(s, obj.gnss_timestamp, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "gnss_time_standard: ";
    YamlStreamer< ::uavcan::equipment::gnss::Fix2::FieldTypes::gnss_time_standard >::stream(s, obj.gnss_time_standard, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "num_leap_seconds: ";
    YamlStreamer< ::uavcan::equipment::gnss::Fix2::FieldTypes::num_leap_seconds >::stream(s, obj.num_leap_seconds, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "longitude_deg_1e8: ";
    YamlStreamer< ::uavcan::equipment::gnss::Fix2::FieldTypes::longitude_deg_1e8 >::stream(s, obj.longitude_deg_1e8, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "latitude_deg_1e8: ";
    YamlStreamer< ::uavcan::equipment::gnss::Fix2::FieldTypes::latitude_deg_1e8 >::stream(s, obj.latitude_deg_1e8, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "height_ellipsoid_mm: ";
    YamlStreamer< ::uavcan::equipment::gnss::Fix2::FieldTypes::height_ellipsoid_mm >::stream(s, obj.height_ellipsoid_mm, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "height_msl_mm: ";
    YamlStreamer< ::uavcan::equipment::gnss::Fix2::FieldTypes::height_msl_mm >::stream(s, obj.height_msl_mm, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "ned_velocity: ";
    YamlStreamer< ::uavcan::equipment::gnss::Fix2::FieldTypes::ned_velocity >::stream(s, obj.ned_velocity, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "sats_used: ";
    YamlStreamer< ::uavcan::equipment::gnss::Fix2::FieldTypes::sats_used >::stream(s, obj.sats_used, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "status: ";
    YamlStreamer< ::uavcan::equipment::gnss::Fix2::FieldTypes::status >::stream(s, obj.status, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "mode: ";
    YamlStreamer< ::uavcan::equipment::gnss::Fix2::FieldTypes::mode >::stream(s, obj.mode, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "sub_mode: ";
    YamlStreamer< ::uavcan::equipment::gnss::Fix2::FieldTypes::sub_mode >::stream(s, obj.sub_mode, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "covariance: ";
    YamlStreamer< ::uavcan::equipment::gnss::Fix2::FieldTypes::covariance >::stream(s, obj.covariance, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "pdop: ";
    YamlStreamer< ::uavcan::equipment::gnss::Fix2::FieldTypes::pdop >::stream(s, obj.pdop, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "ecef_position_velocity: ";
    YamlStreamer< ::uavcan::equipment::gnss::Fix2::FieldTypes::ecef_position_velocity >::stream(s, obj.ecef_position_velocity, level + 1);
}

}

namespace uavcan
{
namespace equipment
{
namespace gnss
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::equipment::gnss::Fix2::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::equipment::gnss::Fix2 >::stream(s, obj, 0);
    return s;
}

} // Namespace gnss
} // Namespace equipment
} // Namespace uavcan

#endif // UAVCAN_EQUIPMENT_GNSS_FIX2_HPP_INCLUDED