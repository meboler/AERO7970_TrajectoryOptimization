# A makefile for Latex using latexmk

all: main.pdf

main.pdf: main.tex
	latexmk -pdf -pdflatex="pdflatex -interactive=nonstopmode"

clean:
	latexmk -c
