# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import os
import sys
sys.path.insert(0, os.path.abspath('../../code/'))

# -- Project information -----------------------------------------------------

project = 'RRT-Dubins'
copyright = '2019, Félicien Cantalloube'
author = 'Félicien Cantalloube'

# The full version, including alpha/beta/rc tags
release = '0.0.1'


# -- General configuration ---------------------------------------------------

autosummary_generate = True
# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = ['sphinx.ext.autodoc',
              'sphinx.ext.imgmath',
              'numpydoc',
              'sphinx.ext.intersphinx',
              'sphinx.ext.coverage',
              'sphinx.ext.viewcode',
              'sphinx.ext.autosummary',
              'matplotlib.sphinxext.plot_directive',
              'sphinx.ext.githubpages']

source_suffix = '.rst'
# The master toctree document.
master_doc = 'index'

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'scipy'
html_theme_path = ['_theme']
html_logo = '_static/scipyshiny_small.png'
html_static_path = ['_static']
html_theme_options = {
    "edit_link": "true",
    "sidebar": "right",
    "scipy_org_logo": "true",
    "rootlinks": [("https://github.com/FelicienC/RRT-Dubins", "Github"),
                  ("https://felicienc.github.io/RRT-Dubins/", "Docs")]
}
