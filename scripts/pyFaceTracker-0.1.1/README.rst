========================
README for pyFaceTracker
========================

FaceTracker is a library for deformable face tracking written in C++ using
OpenCV 2, authored by Jason Saragih and maintained by Kyle McDonald. It is
available free for non-commercial use, and may be redistributed under these
conditions. Please see the LICENSE file for complete details.

**pyfacetracker** is a thin wrapper around FaceTracker. It enables using
FaceTracker while enjoyging the comfort of the Python scripting language.

pyfacetracker is available under the BSD License. This has no effect on
Jason's code, which is available under a separate license.

Installing
==========

Use ``setup.py``::

   python setup.py install


Reading the docs
================

After installing::

   cd doc
   make html

Then, direct your browser to ``build/html/index.html``.


Testing
=======

To run the tests with the interpreter available as ``python``, use::

   cd examples
   python face_image.py path/to/image # path/to/image is a path to a color image


Conditions of use
=================

See the LICENSE file


Contributing
============

For bug reports use the Bitbucket issue tracker.
You can also send wishes, comments, patches, etc. to amitibo@tx.technion.ac.il


Acknowledgement
===============

Thank-you to the people at <http://wingware.com/> for their policy of **free licenses for non-commercial open source developers**.

.. image:: http://wingware.com/images/wingware-logo-180x58.png
