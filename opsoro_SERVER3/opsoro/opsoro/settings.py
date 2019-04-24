import os

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
PACKAGE_ROOT = os.path.abspath(os.path.dirname(__file__))
BASE_DIR = PACKAGE_ROOT


DEBUG = True

# Offline db
DATABASES = {
    'default': {
        'ENGINE': 'django.db.backends.sqlite3',
        'NAME': 'opsorodb',
    }
}
# DATABASES = {
#     'default': {
#         'ENGINE': 'django.db.backends.postgresql_psycopg2',  # Add 'postgresql_psycopg2', 'mysql', 'sqlite3' or 'oracle'.
#         'NAME': 'opsorodb',                      # Or path to database file if using sqlite3.
#         # The following settings are not used with sqlite3:
#         'USER': 'opsoro',
#         'PASSWORD': 'OpsoroIsPro2016',
#         'HOST': 'localhost',                      # Empty for localhost through domain sockets or           '127.0.0.1' for localhost through TCP.
#         'PORT': '',                      # Set to empty string for default.
#     }
# }


# [START dbconfig]
# DATABASES = {
#     'default': {
#         # If you are using Cloud SQL for MySQL rather than PostgreSQL, set
#         # 'ENGINE': 'django.db.backends.mysql' instead of the following.
#         'ENGINE': 'django.db.backends.mysql',
#         'NAME': 'opsorodb',
#         'USER': 'opsoro',
#         'PASSWORD': 'opsoroISpro',
#         'PORT' : '5432',
#     }
# }
# # In the flexible environment, you connect to CloudSQL using a unix socket.
# # Locally, you can use the CloudSQL proxy to proxy a localhost connection
# # to the instance
# DATABASES['default']['HOST'] = '/cloudsql/opsoro-66b06:europe-west1:opsoro'
# if os.getenv('GAE_INSTANCE'):
#     pass
# else:
#     DATABASES['default']['HOST'] = '127.0.0.1'
# [END dbconfig]

ALLOWED_HOSTS = [
    '*', 'localhost', '127.0.0.1:8000', '.opsoro.be',
]

# Local time zone for this installation. Choices can be found here:
# http://en.wikipedia.org/wiki/List_of_tz_zones_by_name
# although not all choices may be available on all operating systems.
# On Unix systems, a value of None will cause Django to use the same
# timezone as the operating system.
# If running in a Windows environment this must be set to the same as your
# system time zone.
TIME_ZONE = 'Europe/Brussels'

# Language code for this installation. All choices can be found here:
# http://www.i18nguy.com/unicode/language-identifiers.html
LANGUAGE_CODE = 'en-us'

SITE_ID = int(os.environ.get('SITE_ID', 1))

# If you set this to False, Django will make some optimizations so as not
# to load the internationalization machinery.
USE_I18N = True

# If you set this to False, Django will not format dates, numbers and
# calendars according to the current locale.
USE_L10N = True

# If you set this to False, Django will not use timezone-aware datetimes.
USE_TZ = True

# Absolute filesystem path to the directory that will hold user-uploaded files.
# Example: '/home/media/media.lawrence.com/media/'
MEDIA_ROOT = os.path.join(PROJECT_ROOT, 'media')

# URL that handles the media served from MEDIA_ROOT. Make sure to use a
# trailing slash.
# Examples: 'http://media.lawrence.com/media/', 'http://example.com/media/'
MEDIA_URL = '/media/'

# Absolute path to the directory static files should be collected to.
# Don't put anything in this directory yourself; store your static files
# in apps' 'static/' subdirectories and in STATICFILES_DIRS.
# Example: '/home/media/media.lawrence.com/static/'
STATIC_ROOT = os.path.join(PROJECT_ROOT, 'opsoro', 'static')
# STATIC_ROOT = 'static/'
# URL prefix for static files.
# Example: 'http://media.lawrence.com/static/'
# STATIC_URL = 'https://storage.googleapis.com/opsoro/static/'
STATIC_URL = '/static/'

# Additional locations of static files
STATICFILES_DIRS = [
    os.path.join(PROJECT_ROOT, 'static', 'dist'),
]

# Adds a hash to the filename for versioning
# STATICFILES_STORAGE = 'django.contrib.staticfiles.storage.ManifestStaticFilesStorage'

# List of finder classes that know how to find static files in
# various locations.
STATICFILES_FINDERS = [
    'django.contrib.staticfiles.finders.FileSystemFinder',
    'django.contrib.staticfiles.finders.AppDirectoriesFinder',
]

# Make this unique, and don't share it with anybody.
SECRET_KEY = '-h^2z9(^q$a-%$i$2k=sdi#20_wi)46)sir=b31rn%*_#-_$k_'

TEMPLATES = [
    {
        'BACKEND': 'django.template.backends.django.DjangoTemplates',
        'DIRS': [
            os.path.join(PACKAGE_ROOT, 'templates'),
        ],
        'APP_DIRS': True,
        'OPTIONS': {
            'debug': DEBUG,
            'context_processors': [
                'django.contrib.auth.context_processors.auth',
                'django.template.context_processors.debug',
                'django.template.context_processors.i18n',
                'django.template.context_processors.media',
                'django.template.context_processors.static',
                'django.template.context_processors.tz',
                'django.template.context_processors.request',
                'django.contrib.messages.context_processors.messages',
                'account.context_processors.account',
                'pinax_theme_bootstrap.context_processors.theme',
                'social_django.context_processors.backends',
                'social_django.context_processors.login_redirect',
            ],
        },
    },
]

MIDDLEWARE = [
    'corsheaders.middleware.CorsMiddleware',
    'django.contrib.sessions.middleware.SessionMiddleware',
    'django.middleware.common.CommonMiddleware',
    'django.middleware.csrf.CsrfViewMiddleware',
    'corsheaders.middleware.CorsPostCsrfMiddleware',
    'django.contrib.auth.middleware.AuthenticationMiddleware',
    'django.contrib.messages.middleware.MessageMiddleware',
    'django.middleware.clickjacking.XFrameOptionsMiddleware',
]

ROOT_URLCONF = 'opsoro.urls'

# Python dotted path to the WSGI application used by Django's runserver.
WSGI_APPLICATION = 'opsoro.wsgi.application'

# Local test
ASGI_APPLICATION = 'opsoro.routing.application'

INSTALLED_APPS = [
    'django.contrib.admin',
    'django.contrib.auth',
    'django.contrib.contenttypes',
    'django.contrib.messages',
    'django.contrib.sessions',
    'django.contrib.sites',
    'django.contrib.staticfiles',
    'django.contrib.humanize',

    # theme
    'bootstrapform',
    'pinax_theme_bootstrap',

    # external
    'account',
    'pinax.eventlog',
    'pinax.webanalytics',
    'pinax.announcements',
    # "pinax.invitations",
    'social_django',
    'corsheaders',
    'stdimage',
    'captcha',
    'easy_thumbnails',
    # "reversion",
    'django_cron',

    # sockets
    'channels',

    # translation
    'rosetta',

    # project
    'opsoro',

    # apps
    'robot',
    'play',
    'documents',
    'shop',
    'analytics',
    'forums',  # Copied from pinax, but changed, because you know... bugs...
    # "teams",  # Copied from pinax, but changed, because you know... bugs...
]

THUMBNAIL_ALIASES = {
    "": {
        "avatar": {"size": (50, 50), "crop": True},
        "avatar-large": {"size": (69, 69), "crop": True},
    },
}

CRON_CLASSES = [
    "opsoro.cron.check_usage",
    # ...
]

# A sample logging configuration. The only tangible logging
# performed by this configuration is to send an email to
# the site admins on every HTTP 500 error when DEBUG=False.
# See http://docs.djangoproject.com/en/dev/topics/logging for
# more details on how to customize your logging configuration.
ADMINS = [('Stan', 'stan.notebaert@gmail.com')]

LOGGING = {
    'version': 1,
    'disable_existing_loggers': False,
    'filters': {
        'require_debug_false': {
            '()': 'django.utils.log.RequireDebugFalse'
        }
    },
    'handlers': {
        'mail_admins': {
            'level': 'ERROR',
            'filters': ['require_debug_false'],
            'class': 'django.utils.log.AdminEmailHandler'
        }
    },
    'loggers': {
        'django.request': {
            'handlers': ['mail_admins'],
            'level': 'ERROR',
            'propagate': True,
        },
    }
}

FIXTURE_DIRS = [os.path.join(PROJECT_ROOT, 'fixtures'), ]

# > SUBDOMAIN -----------------------------------------------------------------------------

# ROOT_URLCONF = 'opsoro.urls'

# A dictionary of urlconf module paths, keyed by their subdomain.
SUBDOMAIN_URLCONFS = {
    # None: 'opsoro.urls',  # no subdomain, e.g. ``example.com``
    # 'www': 'opsoro.urls',
    # 'play': 'opsoro.urls',
}

# < SUBDOMAIN -----------------------------------------------------------------------------
# > MAILING -----------------------------------------------------------------------------
# EMAIL_BACKEND = 'django.core.mail.backends.console.EmailBackend'  # Only writes to the console
# EMAIL_BACKEND = 'django.core.mail.backends.smtp.EmailBackend'
# EMAIL_USE_SSL = True
# EMAIL_HOST = 'localhost'
# EMAIL_PORT = 465

# EMAIL_HOST_USER = 'opsoroah'
# EMAIL_HOST_PASSWORD = 'startupono16'
# SERVER_EMAIL = 'webmaster@opsoro.a2hosted.com'
# DEFAULT_FROM_EMAIL = 'info@opsoro.be'
# EMAIL_HOST_USER = 'webmaster@opsoro.a2hosted.com'
# EMAIL_HOST_PASSWORD = '6RUJV0M6Dik}'

EMAIL_BACKEND = 'django.core.mail.backends.smtp.EmailBackend'
EMAIL_HOST = 'localhost'
EMAIL_USE_SSL = False
EMAIL_USE_TLS = False
EMAIL_PORT = 25
EMAIL_HOST_USER = 'opsoro'
EMAIL_HOST_PASSWORD = ''
DEFAULT_FROM_EMAIL = 'OPSORO <play@opsoro.be>'

THEME_CONTACT_EMAIL = 'platform@opsoro.be'
# < MAILING -----------------------------------------------------------------------------
# > USERS -----------------------------------------------------------------------------
ACCOUNT_OPEN_SIGNUP = True         # Require signup code if False
ACCOUNT_EMAIL_UNIQUE = True
ACCOUNT_EMAIL_CONFIRMATION_REQUIRED = True
ACCOUNT_LOGIN_REDIRECT_URL = LOGIN_REDIRECT_URL = '/'
ACCOUNT_LOGOUT_REDIRECT_URL = LOGIN_URL = '/account/login/'
ACCOUNT_EMAIL_CONFIRMATION_EXPIRE_DAYS = 2
ACCOUNT_USE_AUTH_AUTHENTICATE = True
ACCOUNT_LANGUAGES = [
    ('de', 'Deutsch'),
    ('en', 'English'),
    ('fr', 'francais'),
    ('nl', 'Nederlands'),
]
ACCOUNT_HOOKSET = 'opsoro.hooks.AccountHookSet'
ACCOUNT_DELETION_MARK_CALLBACK = None
ACCOUNT_DELETION_EXPUNGE_CALLBACK = None

AUTHENTICATION_BACKENDS = [
    'account.auth_backends.UsernameAuthenticationBackend',
    'account.auth_backends.EmailAuthenticationBackend',
    'social_core.backends.github.GithubOAuth2',
    'social_core.backends.twitter.TwitterOAuth',
    'social_core.backends.facebook.FacebookOAuth2',
    'social_core.backends.google.GoogleOAuth2',
]

SOCIAL_AUTH_GITHUB_KEY = 'a80197ce04559d2e0126'
SOCIAL_AUTH_GITHUB_SECRET = 'cd8050cb92eb016efeccbd14db64b73cb307d92d'
SOCIAL_AUTH_GITHUB_SCOPE = ['email']

SOCIAL_AUTH_TWITTER_KEY = 'nRZzJ9rKIE9j1vid80OsA0R79'
SOCIAL_AUTH_TWITTER_SECRET = 'Gz07BcFadtlH4CEEtzECC1PM5PFskagT3KDl6JsNHmSkgMFuU1'
SOCIAL_AUTH_TWITTER_SCOPE = ['email']

SOCIAL_AUTH_FACEBOOK_KEY = '1831783030386815'
SOCIAL_AUTH_FACEBOOK_SECRET = '5cdcd50de2ecafdc1390b1ceeeac92db'
SOCIAL_AUTH_FACEBOOK_SCOPE = ['email']
SOCIAL_AUTH_FACEBOOK_PROFILE_EXTRA_PARAMS = {'fields': 'id, name, email'}

SOCIAL_AUTH_GOOGLE_OAUTH2_KEY = '231870910775-u8tje4jtaefv8fnbiet0q49sa3d39me5.apps.googleusercontent.com'
SOCIAL_AUTH_GOOGLE_OAUTH2_SECRET = 'krDfdRTXjURvDdkd-ruZG5zw'
SOCIAL_AUTH_GOOGLE_OAUTH2_SCOPE = ['email']
#
SOCIAL_AUTH_LOGIN_ERROR_URL = '/account/login/'
SOCIAL_AUTH_LOGIN_REDIRECT_URL = '/'
SOCIAL_AUTH_RAISE_EXCEPTIONS = False

CORS_ORIGIN_ALLOW_ALL = True
#
# CORS_ORIGIN_ALLOW_ALL = False
#
# CORS_ORIGIN_WHITELIST = (
#     '*',
# )

CORS_ALLOW_CREDENTIALS = True


RECAPTCHA_PUBLIC_KEY = '6LdzgRYUAAAAAMst6K2QeTJTWFA0CELJw2_sb7rh'
RECAPTCHA_PRIVATE_KEY = '6LdzgRYUAAAAABgqdndgZzoVQOyZ5FyNSDXJuSs9'
# Only checkbox, no complicated stuff
NOCAPTCHA = True

# AUTH_USER_MODEL = 'auth.User'
# FORUMS_HOOKSET = "forums.hooks.ForumsDefaultHookSet"
FORUMS_EDIT_TIMEOUT = dict(minutes=3)


# TEAMS_HOOKSET = "teams.hooks.TeamsDefaultHookSet"
# FORUMS_EDIT_TIMEOUT = dict(minutes=3)

# < USERS -----------------------------------------------------------------------------
# > SHOP -----------------------------------------------------------------------------
# STRIPE_API_KEY = 'sk_test_qLZtSsYXiIVlsZbbzjH25Yd3'
# MESSAGE_STORAGE = 'django.contrib.messages.storage.session.SessionStorage'
# < SHOP -----------------------------------------------------------------------------
# > TRANSLATION -----------------------------------------------------------------------------
ROSETTA_ENABLE_TRANSLATION_SUGGESTIONS = True
YANDEX_TRANSLATE_KEY = 'trnsl.1.1.20161025T084648Z.3d3239b9ae382839.4718c47e89bac27b9c3c7f38fd71c1877def9b45'
AZURE_CLIENT_ID = 'f3f3081f-e1a2-4208-9ba1-b4f088fbc9d4'
AZURE_CLIENT_SECRET = 'wbphgVw4K1YQt0Nf0uuZy6YyriYN5azSobK1txx0wyU'

# < TRANSLATION -----------------------------------------------------------------------------
# > CHANNELS -----------------------------------------------------------------------------

# CHANNEL_LAYERS = {
#     'default': {
#         'BACKEND': 'asgiref.inmemory.ChannelLayer',
#         'ROUTING': 'opsoro.routing.channel_routing',
#     },
# }
CHANNEL_LAYERS = {
    'default': {
        'BACKEND': 'channels_redis.core.RedisChannelLayer',
        'CONFIG': {
            'hosts': [('localhost', 6379)],
        },
    },
}

# < CHANNELS -----------------------------------------------------------------------------
# > SSL -----------------------------------------------------------------------------

#SECURE_PROXY_SSL_HEADER = ('HTTP_X_FORWARDED_PROTO', 'https')
# SECURE_SSL_REDIRECT = False
# SESSION_COOKIE_SECURE = False
# CSRF_COOKIE_SECURE = False

# < SSL ----------------------------------------------------------------------------
# CSRF_USE_SESSIONS = True
CSRF_FAILURE_VIEW = 'opsoro.views.csrf_failure'

# DEBUG ----------------------------------------------------------------------------
EMAIL_BACKEND = 'django.core.mail.backends.console.EmailBackend'  # Only writes to the console
DEBUG = True
