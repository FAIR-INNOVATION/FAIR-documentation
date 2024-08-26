# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'FAIRINO Collaborative Robot User Manual'
copyright = '2022-2024, FAIR Innovation (Suzhou) Robotic System Co.,Ltd.'
author = 'FAIR Innovation (Suzhou) Robotic System Co.,Ltd.'
release = '3.7.4'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = []

templates_path = ['_templates']
exclude_patterns = []



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
html_css_files = ["custom.css"]
html_logo = "_static/logo-en.png"
html_theme_options = {
    "logo_only": True,
    "display_version": False
}