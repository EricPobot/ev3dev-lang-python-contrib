Contribution packages for ev3dev Python language bindings
=========================================================

Additional packages for the Python language binding for ev3dev_.

Automation
----------

A ``fabric`` recipe file (``fabfile.py``) if provided for simplifying repetitive tasks. The
available tasks are :

make_setup
    generates the effective ``setup.py`` file, using the ``setup-template.py`` template.
    It updates the version setting using available git tags. This is done this way so
    that the setup script can be run on the target, where git context is not available.

build
    builds the distribution package, according to the format specified in the
    ``pkg_format`` environment variable. Supported formats are : egg, sdist, wheel

deploy
    deploys the generated archive on the connected target, as identified by the
    ``hosts`` environment variable

install
    executes the installation command on the target

doc
    generates the documentation

demos
    deploy the demonstration programs and their data

make_all
    chains ``make_setup`` to ``install`` tasks. It is the default task if the ``fab``
    command is invoked without a task name

To customize the fabfile without modifying it, you can add settings such as sudo passwords or
hostname overrides in a plain Python file name ``fabconfig.py`` located in the same directory.
If it exists, it is imported after the definitions of environment variables and before the tasks
definitions.

Documentation
-------------

A detailed Sphinx_ generated documentation is provided in the ``docs`` sub-tree. To generate
it, you can either execute ``make html`` from the ``docs`` subdir, or more conveniently
use the ``fab doc`` command which will take care of going to the right place and issue the
``make`` from there.


.. _ev3dev: http://ev3dev.org
.. _Sphinx: http://sphinx-doc.org/
