# Instructions for updating documentation for perls2

## Overview
perls2 uses sphinx and read the docs to automatically build documentation as html files. The documentation is hosted online using github pages.
Currently these instructions should work for sphinx==2.2.1.


## Making updates:
The source files for documentation are stored on the 'docs' page, while github uses 'gh-pages' branch for the website.
We will setup two copies of the repo. One on the 'docs' branch, and one on the 'gh-pages' branch.

### Set up repo with ghpages branch:

1. Clone the perls2 repo to a directory called 'perls2_ghpage'

    `git clone https://github.com/StanfordVL/perls2.git perls2_ghpage`

2. Switch to the gh-pages branch

    `git checkout gh-pages`


### Setup repo with docs Branch:

1. Clone and install the perls2 repo

2. Switch to the docs branch

    `git checkout docs`

3. Edit the docs/Makefile to set the GHREPODIR to the perls2_ghpage root.

    `GHREPODIR = /path/to/perls2_ghpage/`

    This will help ensure that the html files built by sphinx in the docs directory are in the correct location for github pages.

4. Make changes to the documentation in the repo on the docs branch.

5. Build the html files with sphinx

    `cd ~/perls2/docs/`

    `make html ghpages`

    **make sure you use the ghpages command or the website won't work.**

6. Add and commit the changes to the **gh-pages** repo.
    ```
    cd ~/perls2_ghpages
    git add .
    git commit -m "my updates"
    git push
    ```

7. Go to stanfordvl.github.io/perls2/ and hit refresh to see if they were added.

## Updating API
Sphinx auto builds api for classes and libraries. Currently, new classes and modules must be manually added to the index.rst

1. Merge code changes you want to add.

2. Create an .rst file for the module you want to document in the docs directory. It's best to copy an existing one to get the automethod calls and formatting.

3. Add the .rst file path (relative) to the index under API

4. Build the html sphinx files as described above

## Updating webpages.
1. Add an md file to to the sources directory with the web page you want to add.

2. Add a reference to the md file on the index.rst file.

3. Build the html sphinx files as described above.