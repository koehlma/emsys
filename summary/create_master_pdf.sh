#!/bin/bash

set -e

case "${USER}" in
    "user"|"eispin")
        # Feel free to use this on Linux-y systems, Marlene and Maxi.
        echo "Welcome back, Ben."
        SLIDE_PATH="lectures/Lecture_"
        LAYOUT="3x2"
        ;;
    "max")
        echo "Welcome back, Max."
        SLIDE_PATH="/Users/max/Dropbox/University/SS16/EmbeddedSystems/Slides/Lecture_"
        LAYOUT="2x2"
        ;;
    *)
        echo "
 Note: Since I do not give a rat's neck on user friendliness you need
 to manually provide the path to your slides as well as their name
 prefix. Additionally, I need the path to the tex file's directory. And
 since I'm really, really lazy, you have to provide the paths INSIDE
 this file. I'm terrible, eh? Having done so, just remove line 33 and
 actually execute the script.

 We at Schwengers & Wiederhakes Inc. attach utmost importance to
 customer friendliness, so feel free to customize the slide layout to
 your very needs using lines 30-32."
        SLIDE_PATH="/Users/max/Dropbox/University/SS16/EmbeddedSystems/Slides/Lecture_"
        LAYOUT="2x2"
        exit 1
        ;;
esac

pdfjam --a4paper --landscape --fitpaper false --twoside --nup $LAYOUT \
 -o slides.pdf -- \
 ${SLIDE_PATH}2.pdf 11 \
 ${SLIDE_PATH}3.pdf 18,20 \
 ${SLIDE_PATH}4.pdf 35 \
 ${SLIDE_PATH}6.pdf 28 \
 ${SLIDE_PATH}9.pdf 25,27,29,36,39,40-48 \
 ${SLIDE_PATH}11.pdf 33,36,39,40-43,56 \
 ${SLIDE_PATH}14.pdf 36,39,43,45-47,50 \
 ${SLIDE_PATH}16.pdf 38

