#!/bin/bash
gml2gv resources/test.gml | neato -Tsvg | inkscape --pipe -g
