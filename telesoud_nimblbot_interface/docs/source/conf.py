import os
import sys
sys.path.insert(0, os.path.abspath('../../'))

project = 'Telesoud NimblBot Interface'
release = '1.0.0'

extensions = [
        'sphinx.ext.autodoc',
        'sphinx.ext.napoleon',
        'sphinx.ext.viewcode',
        ]

autodoc_default_options = {
        'members': True,
        'member-order': 'bysource',
        'special-members': '__init__',
        'undoc-members': True,
        'exclude-members': '__weakref__',
        'private-members': True,
        }

html_theme = 'pydata_sphinx_theme'
html_theme_options = {
    "logo": {
        "text": "NimblBot Welding",  
    },
    "navbar_align": "left",
    "navbar_center": ["navbar-nav"],
    "navbar_start": ["navbar-logo"],
    "navbar_end": ["theme-switcher", "navbar-icon-links"],
    "icon_links": [
        {
            "name": "GitLab",
            "url": "https://gitlab.nimbl-bot.com/tcourtois/nimblbot-welding",
            "icon": "fa-brands fa-gitlab",
            "type": "fontawesome",
        }
    ],
    "show_nav_level": 2,
}

html_context = {
        "default_mode":"dark"
    }

napoleon_google_docstring = True
napoleon_numpy_docstring = True
napoleon_include_init_with_doc = False
napoleon_preprocess_types = True

# Configuration LaTeX optimisée pour éliminer les pages blanches
latex_engine = 'pdflatex'

latex_elements = {
    'papersize': 'a4paper',
    'pointsize': '10pt',
    'classoptions': 'openany,oneside',  # Évite les pages blanches entre chapitres
    
    'preamble': r'''
% Éviter les pages blanches
\let\cleardoublepage\clearpage

% Marges
\usepackage[margin=2.5cm]{geometry}

% Langue française
\usepackage[french]{babel}

% Éviter l'étirement vertical
\raggedbottom

% Contrôle des veuves et orphelines
\widowpenalty=10000
\clubpenalty=10000

% Améliorer les sauts de page
\usepackage{needspace}
''',
    
    'fncychap': '',  # Supprime les en-têtes fantaisistes
    'printindex': '',
    
    # Configuration Sphinx pour les marges et style
    'sphinxsetup': '''
    hmargin={2.5cm,2.5cm},
    vmargin={2.5cm,2.5cm},
    verbatimwithframe=false,
    ''',
}

latex_documents = [
    ('index', 'nimblbot_welding_doc.tex', "Nimbl'bot Welding Documentation",
     "Nimbl'bot team", 'manual'),
]

# Options LaTeX supplémentaires
latex_show_pagerefs = False
latex_show_urls = 'footnote'
