# Minimal makefile for Sphinx documentation
#

# You can set these variables from the command line.
SPHINXOPTS    =
SPHINXBUILD   = sphinx-build
SOURCEDIR     = .
GHREPODIR 	  = ../../perls2_ghpage/
GHDOCSDIR 	  = ${GHREPODIR}/docs
BUILDDIR      = ${GHDOCSDIR}/build

# Put it first so that "make" without argument is like "make help".
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

.PHONY: help Makefile

# Copy html files from docs/html to docs directory.
# add nojekyll file in case it got deleted
ghpage:
	@cp -r ${BUILDDIR}/html/* ${GHDOCSDIR}
	@rm -r ${BUILDDIR}
	@touch ${GHDOCSDIR}/.nojekyll


# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)
