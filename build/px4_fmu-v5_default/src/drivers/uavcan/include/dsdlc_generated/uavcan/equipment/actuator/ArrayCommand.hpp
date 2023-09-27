/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/bitgar/PX4-Autopilot/src/drivers/uavcan/libuavcan/dsdl/uavcan/equipment/actuator/1010.ArrayCommand.uavcan
 */

#ifndef UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_HPP_INCLUDED
#define UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

#include <uavcan/equipment/actuator/Command.hpp>

/******************************* Source text **********************************
#
# Actuator commands.
# The system supports up to 256 actuators; up to 15 of them can be commanded with one message.
#

Command[<=15] commands
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.equipment.actuator.ArrayCommand
uavcan.equipment.actuator.Command[<=15] commands
******************************************************************************/

#undef commands

namespace uavcan
{
namespace equipment
{
namespace actuator
{

template <int _tmpl>
struct UAVCAN_EXPORT ArrayCommand_
{
    typedef const ArrayCommand_<_tmpl>& ParameterType;
    typedef ArrayCommand_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::Array< ::uavcan::equipment::actuator::Command, ::uavcan::ArrayModeDynamic, 15 > commands;
    };

    enum
    {
        MinBitLen
            = FieldTypes::commands::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::commands::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::commands >::Type commands;

    ArrayCommand_()
        : commands()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<484 == MaxBitLen>::check();
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
    enum { DefaultDataTypeID = 1010 };

    static const char* getDataTypeFullName()
    {
        return "uavcan.equipment.actuator.ArrayCommand";
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
bool ArrayCommand_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        commands == rhs.commands;
}

template <int _tmpl>
bool ArrayCommand_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(commands, rhs.commands);
}

template <int _tmpl>
int ArrayCommand_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::commands::encode(self.commands, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int ArrayCommand_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::commands::decode(self.commands, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature ArrayCommand_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x26EBF643995F91A0ULL);

    FieldTypes::commands::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef ArrayCommand_<0> ArrayCommand;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::uavcan::equipment::actuator::ArrayCommand > _uavcan_gdtr_registrator_ArrayCommand;

}

} // Namespace actuator
} // Namespace equipment
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::equipment::actuator::ArrayCommand >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::equipment::actuator::ArrayCommand::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::equipment::actuator::ArrayCommand >::stream(Stream& s, ::uavcan::equipment::actuator::ArrayCommand::ParameterType obj, const int level)
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
    s << "commands: ";
    YamlStreamer< ::uavcan::equipment::actuator::ArrayCommand::FieldTypes::commands >::stream(s, obj.commands, level + 1);
}

}

namespace uavcan
{
namespace equipment
{
namespace actuator
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::equipment::actuator::ArrayCommand::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::equipment::actuator::ArrayCommand >::stream(s, obj, 0);
    return s;
}

} // Namespace actuator
} // Namespace equipment
} // Namespace uavcan

#endif // UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_HPP_INCLUDED