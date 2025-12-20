#!/bin/bash

function help() {
    echo "Usage: ${0##*/} [-options]"
    echo ""
    echo "where options include:"
    echo "    -a|along            xxx, eg: --along xxx"
}

function do_or_die() {
    eval $* || exit $?
}

function assert_file_exists() {
    local file="$1"
    if [ ! -f "$file" ]; then
        echo "File $1 does not exist!"
        exit 1
    fi
}

ARGS=`getopt -o l:w:o:p: --long length:,width:,output:,path: -- "$@"`
if [ $? != 0 ]; then
    echo "Terminating..."
    help && exit 1
fi

eval set -- "${ARGS}"

while true
do
    case "$1" in
        -p|--path)
            echo "Option path, argument $2";
            arg_path=$2
            shift 2
            ;;
        -o|--output)
            echo "Option output, argument $2";
            arg_output=$2
            shift 2
            ;;
        -l|--length)
            echo "Option length, argument $2";
            arg_length=$2
            shift 2
            ;;
        -w|--width)
            echo "Option width, argument $2";
            arg_width=$2
            shift 2
            ;;
        --)
            shift
            break
            ;;
        -h)
            help && exit 0
            ;;
        *)
            echo "Internal error!"
            help && exit 1
            ;;
    esac
done

for arg in $@
do
    echo "processing $arg"
done

PKG_PATH=${arg_path:-"."}
OUTPATH=${arg_output:-"/tmp/world_gen"}
L=${arg_length:-"10"}
W=${arg_width:-"10"}

rm -rf ${OUTPATH}
mkdir -p ${OUTPATH}

MODELS_DIR="$PKG_PATH/models"
WORLDS_DIR="$PKG_PATH/worlds"

echo "Generating world in $OUTPATH..."

# Generate ground
do_or_die erb length=$L width=$W "$MODELS_DIR/parametric_ground/model.sdf.erb" > "$OUTPATH/ground.sdf"
assert_file_exists "$OUTPATH/ground.sdf"
# Generate walls
do_or_die erb length=$L "$MODELS_DIR/parametric_wall/model.sdf.erb" > "$OUTPATH/wall_l.sdf"
assert_file_exists "$OUTPATH/wall_l.sdf"
do_or_die erb length=$W "$MODELS_DIR/parametric_wall/model.sdf.erb" > "$OUTPATH/wall_w.sdf"
assert_file_exists "$OUTPATH/wall_w.sdf"
# Generate world
do_or_die erb length=$L width=$W \
    ground_uri="$OUTPATH/ground.sdf" \
    wall_l_uri="$OUTPATH/wall_l.sdf" \
    wall_w_uri="$OUTPATH/wall_w.sdf" \
    "$WORLDS_DIR/empty_room/world.sdf.erb" > "$OUTPATH/world.sdf"
assert_file_exists "$OUTPATH/world.sdf"

echo "Done!"
exit 0