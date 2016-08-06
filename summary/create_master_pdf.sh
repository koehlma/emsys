#!/bin/bash
set -e # yeah, I learned something.

SLIDE_PATH="/Users/max/Dropbox/University/SS16/EmbeddedSystems/Slides/Lecture_"
TEX_PATH="/Users/max/Workspace/embeddedsystems/summary"
LAYOUT="2x2"

echo "Note: Since I do not give a rat's neck on user friendlyness you need to manually provide the path to your slides as well as their name prefix."
echo "Additionally, I need the path to the tex file's directory."
echo "And since I'm really, really lazy, you have to provide the paths INSIDE this file. I'm terrible, eh?"
echo "Having done so, just remove line 15 and actually execute the script."
echo
echo "We at Schwengers Inc. attach uttermost importance to customer friendlyness, so feel free to customize the slide layout to your very needs using line 6."

false;

cd "${TEX_PATH}"

# three invocations plus one for good measure.
pdflatex summary.tex
pdflatex summary.tex
pdflatex summary.tex
pdflatex summary.tex

pdfjam --a4paper --landscape --fitpaper false --twoside --nup $LAYOUT -o wow_thats_awesome.pdf -- ${SLIDE_PATH}2.pdf 11 ${SLIDE_PATH}3.pdf 18,20 ${SLIDE_PATH}6.pdf 28 ${SLIDE_PATH}9.pdf 25,27,29,36,39,40-48 ${SLIDE_PATH}11.pdf 33,36,39,40-43,56 ${SLIDE_PATH}14.pdf 36,39,43,45-47,50 ${SLIDE_PATH}16.pdf 38

pdfjoin -o wow_thats_more_awesomer.pdf summary.pdf wow_thats_awesome.pdf 
