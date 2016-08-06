SOURCES:=$(wildcard *.tex)
TMPFILES:=$(addprefix summary.,aux log pdf toc mw out) slides.pdf combined.pdf

all: combined.pdf

combined.pdf: summary.pdf slides.pdf
	pdfjoin -o $@ $^

summary.pdf: summary.tex ${SOURCES}
# three invocations plus one for good measure.
	pdflatex -halt-on-error summary.tex
	pdflatex -halt-on-error summary.tex
	pdflatex -halt-on-error summary.tex
	pdflatex -halt-on-error summary.tex

slides.pdf:
	./create_master_pdf.sh

.PHONY: clean
clean:
	rm -f ${TMPFILES}

# For debugging the Makefile
.PHONY: say
say:
	@echo "Sources:\t$(SOURCES)"
	@echo "tmpfiles:\t$(TMPFILES)"
