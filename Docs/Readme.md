This directory contains source files for the documentation as well as the generated doxygen output.

# Directory Layout

`. (Docs root)` High level content and format files. (E.g. css, header, footer.)

`./Extern` API documentation that is located outside the source files.  When the API documentation gets too big or complex for the header and source files, it goes in this directory.

`./Images` Images related to the documentation.

`./html` The target for the Doxygen build.  (Created during the build process.)
    
# Documentation Style

One of the requirements for the API documentation is that it
has the minimum possible impact on the declarations in the
header files.  So, in general, the header file declarations only
contain summary documentation.  The detail documentation
is placed as follows:

1.  If an element is defined in a cpp file, then place
    the detail documentation in the source file.
2.  If an element does not have an associated cpp file, then
    place the detail documentation at the end of the header file.
3.  If there is a lot of detail documentation cluttering up
    the end of a header file, then the content is moved to 
	a separate file in the Extern directory.

# Building the Documentation

1.  Download and install the appropriate Doxygen version.  (See the first line in the Doxyfile for the current version.)
2.  Run "doxygen" in the project root directory. (The location of the Doxyfile.)  No arguments are required.

The generated html files will be located in the /Docs/html directory.