#!/bin/bash

for FILE in *.pdf
do
	gs -o fixed-$FILE -sDEVICE=pdfwrite -dColorConversionStrategy=/sRGB -dProcessColorModel=/DeviceRGB -dCompatibilityLevel=1.4 $FILE
	mv fixed-$FILE $FILE
done
