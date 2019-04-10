#!/bin/bash
##
# @file
# @author Denise Ratasich
# @date 25.05.2016
#
# @brief Sends messages to a CAN interface periodically.
##

verbose=0

function usage()
{
    cat <<EOF
  Usage: $0 [-i INTERFACE] [-v] PERIOD ID VALUE

  Sends a CAN message periodically to the specified interface.

  Options:
    -i INTERFACE select interface (see ifconfig), default is can0
    -v increases verbosity level (enables debug outputs)

  Arguments:
    PERIOD period in s the message should be sent.
    ID CAN message ID; either 3 or 8 values; 11 or 29 bits; e.g., 123
    VALUE value of the CAN message, 8 bytes in hex values, e.g.,
      0102030405060708
EOF
    exit 1
}

function debug()
{
    if [ $verbose -ge 1 ]
    then
        tstamp=$(date +%s.%N)
        echo "[debug][$tstamp][$0] $1" >&2
    fi
}

function info()
{
    tstamp=$(date +%s.%N)
    echo "[ info][$tstamp][$0] $1" >&2
}

function error()
{
    tstamp=$(date +%s.%N)
    echo "[error][$tstamp][$0] $1" >&2
}

# default options
interface="can0"

# parse args
coptions=0 # counts arguments that are part of options
while getopts "i:v" opt; do
    case $opt in
        i)
            interface=$OPTARG
            coptions=$(($coptions+2))
            ;;
        v)
            verbose=$(($verbose+1))
            coptions=$(($coptions+1))
            ;;
        \?)
            echo "Invalid option: -$OPTARG" >&2
            usage
            ;;
        :)
            echo "Option -$OPTARG requires an argument." >&2
            usage
            ;;
    esac
done

args=$(($# - $coptions))
if [ $args -ne 3 ]
then
    error "Too few arguments."
    usage
fi

period=${@:$OPTIND:1}
id=${@:$OPTIND+1:1}
value=${@:$OPTIND+2:1}


# send messages forever
debug "execute: cansend $interface $id#$value"
cansend $interface $id#$value
if [ $? -ne 0 ]
then
    error "cansend failed."
    exit 1
fi

while [ 1 ]
do
    sleep $period
    debug "execute: cansend $interface $id#$value"
    cansend $interface $id#$value
done
