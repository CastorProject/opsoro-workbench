import re
from functools import partial

from account.conf import settings
from account.forms import PasswordField
from account.hooks import hookset
from account.models import EmailAddress
from account.utils import get_user_lookup_kwargs
from captcha.fields import ReCaptchaField
from django import forms
from django.contrib import auth
from django.contrib.auth import get_user_model
from django.contrib.auth.models import User
from django.utils.encoding import force_text
from django.utils.translation import ugettext_lazy as _

from opsoro.models import Profile
from shop.models import NewsLetter

try:
    from collections import OrderedDict
except ImportError:
    OrderedDict = None

alnum_re = re.compile(r"^\w+$")

LANGUAGES = [("en", "English"), ("nl", "Nederlands")]


class ProfileForm(forms.Form):
    username = forms.CharField(label=_("Username"), widget=forms.TextInput(attrs={'placeholder': _('Username')}), required=True)
    first_name = forms.CharField(label=_("First Name"), widget=forms.TextInput(attrs={'placeholder': _('First Name')}), required=False)
    last_name = forms.CharField(label=_("Last Name"), widget=forms.TextInput(attrs={'placeholder': _('Last Name')}), required=False)
    email = forms.EmailField(label=_("Email"), widget=forms.EmailInput(attrs={'placeholder': _('Email')}), required=True)
    address = forms.CharField(label=_("Address"), widget=forms.TextInput(attrs={'placeholder': _('Street + nr')}), required=False)
    postal = forms.CharField(label=_("Postal Code"), widget=forms.TextInput(attrs={'placeholder': _('Postal Code')}), required=False)
    city = forms.CharField(label=_("City"), widget=forms.TextInput(attrs={'placeholder': _('City')}), required=False)
    country = forms.CharField(label=_("Country"), widget=forms.TextInput(attrs={'placeholder': _('Country')}), required=False)
    birth_date = forms.CharField(label=_("Birthday"), widget=forms.TextInput(attrs={'placeholder': _('Birthday (yyyy-mm-dd)')}), required=False)

    avatar = forms.FileField(required=False)
    avatar_clear = forms.BooleanField(label=_("Remove avatar"), required=False)

    captcha = ReCaptchaField()

    if settings.USE_I18N:
        language = forms.ChoiceField(label=_("Language"), choices=LANGUAGES, required=False)


class DisputeForm(forms.Form):
    message = forms.CharField(label=_("Message"), widget=forms.TextInput(attrs={'placeholder': _('Explain your problem')}), required=True)


class NewsLetterForm(forms.Form):
    email = forms.EmailField(label=_("Email"), widget=forms.EmailInput(attrs={'placeholder': _('Email')}), required=True)

    def clean(self):
        if not self.errors:
            if 'email' in self.cleaned_data:
                nl = NewsLetter()
                nl.email = self.cleaned_data['email']
                nl.save()
                return self.cleaned_data['email']
        raise forms.ValidationError(_("Invalid email address."))


class SignupForm(forms.Form):

    username = forms.CharField(label=_("Username"), max_length=60, widget=forms.TextInput(), required=True)
    password = PasswordField(label=_("Password"), strip=settings.ACCOUNT_PASSWORD_STRIP)
    password_confirm = PasswordField(label=_("Password (again)"), strip=settings.ACCOUNT_PASSWORD_STRIP)
    email = forms.EmailField(label=_("Email"), widget=forms.TextInput(), required=True)

    code = forms.CharField(max_length=64, required=False, widget=forms.HiddenInput())

    captcha = ReCaptchaField()
    captcha.validate = lambda x: True

    def clean_username(self):
        if not alnum_re.search(self.cleaned_data["username"]):
            raise forms.ValidationError(_("Usernames can only contain letters, numbers and underscores."))
        User = get_user_model()
        lookup_kwargs = get_user_lookup_kwargs({
            "{username}__iexact": self.cleaned_data["username"]
        })
        qs = User.objects.filter(**lookup_kwargs)
        if not qs.exists():
            return self.cleaned_data["username"]
        raise forms.ValidationError(_("This username is already taken. Please choose another."))

    def clean_email(self):
        value = self.cleaned_data["email"]
        qs = EmailAddress.objects.filter(email__iexact=value)
        if not qs.exists() or not settings.ACCOUNT_EMAIL_UNIQUE:
            return value
        raise forms.ValidationError(_("A user is registered with this email address."))

    def clean(self):
        if "password" in self.cleaned_data and "password_confirm" in self.cleaned_data:
            if self.cleaned_data["password"] != self.cleaned_data["password_confirm"]:
                raise forms.ValidationError(_("You must type the same password each time."))
        return self.cleaned_data


class LoginForm(forms.Form):

    password = PasswordField(label=_("Password"), strip=settings.ACCOUNT_PASSWORD_STRIP)
    remember = forms.BooleanField(label=_("Remember Me"), required=False)
    captcha = ReCaptchaField()
    captcha.validate = lambda x: True

    user = None

    def clean(self):
        if self._errors:
            return
        user = auth.authenticate(**self.user_credentials())
        if user:
            if user.is_active:
                self.user = user
            else:
                raise forms.ValidationError(_("This account is inactive."))
        else:
            self.field_order = ["username", "password", "remember"]
            raise forms.ValidationError(self.authentication_fail_message)
        return self.cleaned_data

    def user_credentials(self):
        return hookset.get_user_credentials(self, self.identifier_field)


class LoginUsernameForm(LoginForm):

    username = forms.CharField(label=_("Username /email"), max_length=60)
    authentication_fail_message = _("The username/email and password you specified did not match.")
    identifier_field = "username"

    def __init__(self, *args, **kwargs):
        super(LoginUsernameForm, self).__init__(*args, **kwargs)
        field_order = ["username", "password", "remember", "captcha"]
        if not OrderedDict or hasattr(self.fields, "keyOrder"):
            self.fields.keyOrder = field_order
        else:
            self.fields = OrderedDict((k, self.fields[k]) for k in field_order)


class PasswordResetForm(forms.Form):

    email = forms.EmailField(label=_("Email"), required=True)

    captcha = ReCaptchaField()
    captcha.validate = lambda x: True

    def clean_email(self):
        value = self.cleaned_data["email"]
        if not EmailAddress.objects.filter(email__iexact=value).exists():
            raise forms.ValidationError(_("Email address can not be found."))
        return value
