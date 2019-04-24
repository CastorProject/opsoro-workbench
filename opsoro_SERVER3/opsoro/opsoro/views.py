from __future__ import unicode_literals

from account import signals
from account.conf import settings
# from account.forms import PasswordResetTokenForm
from account.hooks import hookset
from account.mixins import LoginRequiredMixin
from account.models import (Account, AccountDeletion, EmailAddress,
                            EmailConfirmation, PasswordHistory, SignupCode)
from account.utils import default_redirect, get_form_data
from account.views import LogoutView, PasswordMixin
from django.contrib import auth, messages
from django.contrib.auth import get_user_model, update_session_auth_hash
from django.contrib.auth.decorators import login_required
from django.contrib.auth.forms import (AdminPasswordChangeForm,
                                       PasswordChangeForm)
# from django.contrib.auth.hashers import make_password
from django.contrib.auth.models import User
from django.contrib.auth.tokens import default_token_generator
from django.contrib.sites.shortcuts import get_current_site
from django.urls import reverse
from django.http import Http404, HttpResponse, HttpResponseForbidden
from django.shortcuts import get_object_or_404, redirect, render
from django.utils import timezone
from django.utils.http import base36_to_int, int_to_base36
from django.utils.translation import ugettext_lazy as _
# from django.views.generic.base import TemplateResponseMixin, View
from django.views.generic.edit import FormView

from opsoro.compat import is_authenticated, reverse
from opsoro.forms import *
from shop.models import DisputeMessage, Order


def csrf_failure(request, reason=""):
    ctx = {'message': ''}
    return render(request, '400.html', ctx)


class SettingsView(LoginRequiredMixin, FormView):

    template_name = "account/settings.html"
    form_class = ProfileForm
    redirect_field_name = "next"
    messages = {
        "settings_updated": {
            "level": messages.SUCCESS,
            "text": _("Account settings updated.")
        },
    }

    def get_form_class(self):
        # @@@ django: this is a workaround to not having a dedicated method
        # to initialize self with a request in a known good state (of course
        # this only works with a FormView)
        self.primary_email_address = EmailAddress.objects.get_primary(self.request.user)
        return super(SettingsView, self).get_form_class()

    def get_initial(self):
        initial = super(SettingsView, self).get_initial()
        if self.primary_email_address:
            initial['username'] = self.request.user.username
            initial['first_name'] = self.request.user.first_name
            initial['last_name'] = self.request.user.last_name
            initial["email"] = self.primary_email_address.email
            initial['address'] = self.request.user.profile.address
            initial['postal'] = self.request.user.profile.postal
            initial['city'] = self.request.user.profile.city
            initial['country'] = self.request.user.profile.country
            initial['birth_date'] = self.request.user.profile.birth_date
            initial["language"] = self.request.user.account.language
            # initial["stripe_id"] = self.request.user.profile.stripe_id

            return initial

    def form_valid(self, form):
        self.update_settings(form)
        if self.messages.get("settings_updated"):
            messages.add_message(
                self.request,
                self.messages["settings_updated"]["level"],
                self.messages["settings_updated"]["text"]
            )
        return redirect(self.get_success_url())

    def update_settings(self, form):
        self.update_user(form)
        self.update_account(form)
        self.update_profile(form)

    def update_user(self, form):
        fields = {}
        if 'username' in form.cleaned_data:
            if self.request.user.username != form.cleaned_data['username']:
                if not User.objects.filter(username=form.cleaned_data['username']):
                    fields['username'] = form.cleaned_data['username']
                else:
                    messages.add_message(self.request, messages.ERROR, "This username is already in use, please pick another")

        if 'first_name' in form.cleaned_data:
            fields['first_name'] = form.cleaned_data['first_name']
        if 'last_name' in form.cleaned_data:
            fields['last_name'] = form.cleaned_data['last_name']
        if fields:
            user = self.request.user
            for k, v in fields.items():
                setattr(user, k, v)
            user.save()
        self.update_email(form)

    def update_email(self, form, confirm=None):
        user = self.request.user
        if confirm is None:
            confirm = settings.ACCOUNT_EMAIL_CONFIRMATION_EMAIL
        # @@@ handle multiple emails per user
        email = form.cleaned_data["email"].strip()
        if not self.primary_email_address:
            user.email = email
            EmailAddress.objects.add_email(self.request.user, email, primary=True, confirm=confirm)
            user.save()
        else:
            if email != self.primary_email_address.email:
                self.primary_email_address.change(email, confirm=confirm)

    def get_context_data(self, **kwargs):
        ctx = super(SettingsView, self).get_context_data(**kwargs)
        redirect_field_name = self.get_redirect_field_name()
        ctx.update({
            "redirect_field_name": redirect_field_name,
            "redirect_field_value": self.request.POST.get(redirect_field_name, self.request.GET.get(redirect_field_name, "")),
        })
        return ctx

    def update_account(self, form):
        fields = {}
        if "timezone" in form.cleaned_data:
            fields["timezone"] = form.cleaned_data["timezone"]
        if "language" in form.cleaned_data:
            fields["language"] = form.cleaned_data["language"]
        if fields:
            account = self.request.user.account
            for k, v in fields.items():
                setattr(account, k, v)
            account.save()

    def update_profile(self, form):
        if form.is_valid():
            fields = {}
            if 'address' in form.cleaned_data and form.cleaned_data['address'] != '':
                fields['address'] = form.cleaned_data['address']
            if 'postal' in form.cleaned_data and form.cleaned_data['postal'] != '':
                fields['postal'] = form.cleaned_data['postal']
            if 'city' in form.cleaned_data and form.cleaned_data['city'] != '':
                fields['city'] = form.cleaned_data['city']
            if 'country' in form.cleaned_data and form.cleaned_data['country'] != '':
                fields['country'] = form.cleaned_data['country']
            if 'birth_date' in form.cleaned_data and form.cleaned_data['birth_date'] != '':
                fields['birth_date'] = form.cleaned_data['birth_date']

            if 'avatar_clear' in form.cleaned_data and form.cleaned_data['avatar_clear'] is True:
                fields['avatar'] = None
            if 'avatar' in form.cleaned_data and form.cleaned_data['avatar'] is not None:
                fields['avatar'] = form.cleaned_data['avatar']

            if fields:
                profile = self.request.user.profile
                for k, v in fields.items():
                    setattr(profile, k, v)
                profile.save()

    def get_redirect_field_name(self):
        return self.redirect_field_name

    def get_success_url(self, fallback_url=None, **kwargs):
        if fallback_url is None:
            fallback_url = settings.ACCOUNT_SETTINGS_REDIRECT_URL
        kwargs.setdefault("redirect_field_name", self.get_redirect_field_name())
        return default_redirect(self.request, fallback_url, **kwargs)


def index(request, orderId=None):

    if orderId is not None:
        order = Order.objects.get(order_id=orderId)
        if request.method == "POST":
            form = DisputeForm(request.POST)
            if 'message' in form.data and form.data['message'] != '':
                message = form.data['message']
                disp = DisputeMessage()
                disp.create(order, message, request.user)
                messages.add_message(request, messages.SUCCESS, 'Dispute has been opened, we will look into it!')
        else:
            react = False
            if order.status == 'x':
                if order.dispute_messages.last().user == request.user:
                    if order.dispute_messages.count() < 5:
                        react = True
                else:
                    react = True
            else:
                react = True

            return render(request, 'account/orderDetail.html', {'order': order, 'form': DisputeForm(), 'can_react': react})

    orders = request.user.orders.all().order_by('-order_date')
    return render(request, 'account/orders.html', {'orders': orders})


def confirm_received(request, orderId=None):
    if orderId is not None:
        if request.method == "GET":
            order = Order.objects.get(order_id=orderId)
            order.received()

    return index(request, orderId)


@login_required
def password(request):
    if request.user.has_usable_password():
        PasswordForm = PasswordChangeForm
    else:
        PasswordForm = AdminPasswordChangeForm

    if request.method == 'POST':
        form = PasswordForm(request.user, request.POST)
        if form.is_valid():
            form.save()
            update_session_auth_hash(request, form.user)
            messages.success(request, 'Your password was successfully updated!')
            return redirect('/account/settings/')
        else:
            messages.error(request, 'Please correct the error below.')
    else:
        form = PasswordForm(request.user)
    return render(request, 'account/password_change.html', {'form': form})


class SignupView(PasswordMixin, FormView):

    template_name = "account/signup.html"
    template_name_ajax = "account/ajax/signup.html"
    template_name_email_confirmation_sent = "account/email_confirmation_sent.html"
    template_name_email_confirmation_sent_ajax = "account/ajax/email_confirmation_sent.html"
    template_name_signup_closed = "account/signup_closed.html"
    template_name_signup_closed_ajax = "account/ajax/signup_closed.html"
    form_class = SignupForm
    form_kwargs = {}
    form_password_field = "password"
    redirect_field_name = "next"
    identifier_field = "username"
    messages = {
        "email_confirmation_sent": {
            "level": messages.INFO,
            "text": _("Confirmation email sent to {email}.")
        },
        "invalid_signup_code": {
            "level": messages.WARNING,
            "text": _("The code {code} is invalid.")
        }
    }
    fallback_url_setting = "ACCOUNT_SIGNUP_REDIRECT_URL"

    def __init__(self, *args, **kwargs):
        self.created_user = None
        kwargs["signup_code"] = None
        super(SignupView, self).__init__(*args, **kwargs)

    def dispatch(self, request, *args, **kwargs):
        self.request = request
        self.args = args
        self.kwargs = kwargs
        self.setup_signup_code()
        return super(SignupView, self).dispatch(request, *args, **kwargs)

    def setup_signup_code(self):
        code = self.get_code()
        if code:
            try:
                self.signup_code = SignupCode.check_code(code)
            except SignupCode.InvalidCode:
                self.signup_code = None
            self.signup_code_present = True
        else:
            self.signup_code = None
            self.signup_code_present = False

    def get(self, *args, **kwargs):
        if is_authenticated(self.request.user):
            return redirect(default_redirect(self.request, settings.ACCOUNT_LOGIN_REDIRECT_URL))
        if not self.is_open():
            return self.closed()
        return super(SignupView, self).get(*args, **kwargs)

    def post(self, *args, **kwargs):
        if is_authenticated(self.request.user):
            raise Http404()
        if not self.is_open():
            return self.closed()
        return super(SignupView, self).post(*args, **kwargs)

    def get_initial(self):
        initial = super(SignupView, self).get_initial()
        if self.signup_code:
            initial["code"] = self.signup_code.code
            if self.signup_code.email:
                initial["email"] = self.signup_code.email
        return initial

    def get_template_names(self):
        if self.request.is_ajax():
            return [self.template_name_ajax]
        else:
            return [self.template_name]

    def get_form_kwargs(self):
        kwargs = super(SignupView, self).get_form_kwargs()
        kwargs.update(self.form_kwargs)
        return kwargs

    def form_invalid(self, form):
        signals.user_sign_up_attempt.send(
            sender=SignupForm,
            username=get_form_data(form, self.identifier_field),
            email=get_form_data(form, "email"),
            result=form.is_valid()
        )
        return super(SignupView, self).form_invalid(form)

    def form_valid(self, form):
        self.created_user = self.create_user(form, commit=False)
        # prevent User post_save signal from creating an Account instance
        # we want to handle that ourself.
        self.created_user._disable_account_creation = True
        self.created_user.save()
        self.use_signup_code(self.created_user)
        email_address = self.create_email_address(form)
        if settings.ACCOUNT_EMAIL_CONFIRMATION_REQUIRED and not email_address.verified:
            self.created_user.is_active = False
            self.created_user.save()
        self.create_account(form)
        self.create_password_history(form, self.created_user)
        self.after_signup(form)
        if settings.ACCOUNT_EMAIL_CONFIRMATION_EMAIL and not email_address.verified:
            self.send_email_confirmation(email_address)

        try:
            profile = self.created_user.profile
            profile.ip_created = self.request.META.get('REMOTE_ADDR')
            profile.save()
        except Exception as e:
            print(e)

        if settings.ACCOUNT_EMAIL_CONFIRMATION_REQUIRED and not email_address.verified:
            return self.email_confirmation_required_response()
        else:
            show_message = [
                settings.ACCOUNT_EMAIL_CONFIRMATION_EMAIL,
                self.messages.get("email_confirmation_sent"),
                not email_address.verified
            ]
            if all(show_message):
                messages.add_message(
                    self.request,
                    self.messages["email_confirmation_sent"]["level"],
                    self.messages["email_confirmation_sent"]["text"].format(**{
                        "email": form.cleaned_data["email"]
                    })
                )
            # attach form to self to maintain compatibility with login_user
            # API. this should only be relied on by d-u-a and it is not a stable
            # API for site developers.
            self.form = form
            self.login_user()
        return redirect(self.get_success_url())

    def create_user(self, form, commit=True, model=None, **kwargs):
        User = model
        if User is None:
            User = get_user_model()
        user = User(**kwargs)
        username = form.cleaned_data.get("username")
        if username is None:
            username = self.generate_username(form)
        user.username = username
        user.email = form.cleaned_data["email"].strip()
        password = form.cleaned_data.get("password")
        if password:
            user.set_password(password)
        else:
            user.set_unusable_password()
        if commit:
            user.save()
        return user

    def create_account(self, form):
        return Account.create(request=self.request, user=self.created_user, create_email=False)

    def generate_username(self, form):
        raise NotImplementedError(
            "Unable to generate username by default. "
            "Override SignupView.generate_username in a subclass."
        )

    def create_email_address(self, form, **kwargs):
        kwargs.setdefault("primary", True)
        kwargs.setdefault("verified", False)
        if self.signup_code:
            kwargs["verified"] = self.created_user.email == self.signup_code.email if self.signup_code.email else False
        return EmailAddress.objects.add_email(self.created_user, self.created_user.email, **kwargs)

    def use_signup_code(self, user):
        if self.signup_code:
            self.signup_code.use(user)

    def send_email_confirmation(self, email_address):
        email_address.send_confirmation(site=get_current_site(self.request))

    def after_signup(self, form):
        signals.user_signed_up.send(sender=SignupForm, user=self.created_user, form=form)

    def login_user(self):
        user = auth.authenticate(**self.user_credentials())
        auth.login(self.request, user)
        self.request.session.set_expiry(0)

    def user_credentials(self):
        return hookset.get_user_credentials(self.form, self.identifier_field)

    def get_code(self):
        return self.request.POST.get("code", self.request.GET.get("code"))

    def is_open(self):
        if self.signup_code:
            return True
        else:
            if self.signup_code_present:
                if self.messages.get("invalid_signup_code"):
                    messages.add_message(
                        self.request,
                        self.messages["invalid_signup_code"]["level"],
                        self.messages["invalid_signup_code"]["text"].format(**{
                            "code": self.get_code(),
                        })
                    )
        return settings.ACCOUNT_OPEN_SIGNUP

    def email_confirmation_required_response(self):
        if self.request.is_ajax():
            template_name = self.template_name_email_confirmation_sent_ajax
        else:
            template_name = self.template_name_email_confirmation_sent
        response_kwargs = {
            "request": self.request,
            "template": template_name,
            "context": {
                "email": self.created_user.email,
                "success_url": self.get_success_url(),
            }
        }
        return self.response_class(**response_kwargs)

    def closed(self):
        if self.request.is_ajax():
            template_name = self.template_name_signup_closed_ajax
        else:
            template_name = self.template_name_signup_closed
        response_kwargs = {
            "request": self.request,
            "template": template_name,
        }
        return self.response_class(**response_kwargs)


class LoginView(FormView):

    template_name = "account/login.html"
    template_name_ajax = "account/ajax/login.html"
    form_class = LoginUsernameForm
    form_kwargs = {}
    redirect_field_name = "next"

    def get(self, *args, **kwargs):
        if is_authenticated(self.request.user):
            return redirect(self.get_success_url())
        return super(LoginView, self).get(*args, **kwargs)

    def get_template_names(self):
        if self.request.is_ajax():
            return [self.template_name_ajax]
        else:
            return [self.template_name]

    def get_context_data(self, **kwargs):
        ctx = super(LoginView, self).get_context_data(**kwargs)
        redirect_field_name = self.get_redirect_field_name()
        ctx.update({
            "redirect_field_name": redirect_field_name,
            "redirect_field_value": self.request.POST.get(redirect_field_name, self.request.GET.get(redirect_field_name, "")),
        })
        return ctx

    def get_form_kwargs(self):
        kwargs = super(LoginView, self).get_form_kwargs()
        kwargs.update(self.form_kwargs)
        return kwargs

    def form_invalid(self, form):
        signals.user_login_attempt.send(
            sender=LoginView,
            username=get_form_data(form, form.identifier_field),
            result=form.is_valid()
        )
        return super(LoginView, self).form_invalid(form)

    def form_valid(self, form):
        self.login_user(form)
        self.after_login(form)
        return redirect(self.get_success_url())

    def after_login(self, form):
        signals.user_logged_in.send(sender=LoginView, user=form.user, form=form)

    def get_success_url(self, fallback_url=None, **kwargs):
        if fallback_url is None:
            fallback_url = settings.ACCOUNT_LOGIN_REDIRECT_URL
        kwargs.setdefault("redirect_field_name", self.get_redirect_field_name())
        return default_redirect(self.request, fallback_url, **kwargs)

    def get_redirect_field_name(self):
        return self.redirect_field_name

    def login_user(self, form):
        auth.login(self.request, form.user)
        expiry = settings.ACCOUNT_REMEMBER_ME_EXPIRY if form.cleaned_data.get("remember") else 0
        self.request.session.set_expiry(expiry)
        try:
            profile = self.request.user.profile
            profile.ip_last_login = self.request.META.get('REMOTE_ADDR')
            profile.save()
        except Exception as e:
            print(e)


class PasswordResetView(FormView):

    template_name = "account/password_reset.html"
    template_name_sent = "account/password_reset_sent.html"
    form_class = PasswordResetForm
    token_generator = default_token_generator

    def get_context_data(self, **kwargs):
        context = super(PasswordResetView, self).get_context_data(**kwargs)
        if self.request.method == "POST" and "resend" in self.request.POST:
            context["resend"] = True
        return context

    def form_valid(self, form):
        self.send_email(form.cleaned_data["email"])
        response_kwargs = {
            "request": self.request,
            "template": self.template_name_sent,
            "context": self.get_context_data(form=form)
        }
        return self.response_class(**response_kwargs)

    def send_email(self, email):
        User = get_user_model()
        protocol = getattr(settings, "DEFAULT_HTTP_PROTOCOL", "http")
        current_site = get_current_site(self.request)
        email_qs = EmailAddress.objects.filter(email__iexact=email)
        for user in User.objects.filter(pk__in=email_qs.values("user")):
            uid = int_to_base36(user.id)
            token = self.make_token(user)
            password_reset_url = "{0}://{1}{2}".format(
                protocol,
                self.request.get_host(),
                reverse("account_password_reset_token", kwargs=dict(uidb36=uid, token=token))
            )
            ctx = {
                "user": user,
                "current_site": current_site,
                "password_reset_url": password_reset_url,
            }
            hookset.send_password_reset_email([user.email], ctx)

    def make_token(self, user):
        return self.token_generator.make_token(user)


class DeleteView(LogoutView):

    template_name = "account/delete.html"
    messages = {
        "account_deleted": {
            "level": messages.WARNING,
            "text": _("Your account is now inactive and your data will be expunged in the next {expunge_hours} hours.")
        },
    }

    def post(self, *args, **kwargs):
        AccountDeletion.mark(self.request.user)
        auth.logout(self.request)
        messages.add_message(
            self.request,
            self.messages["account_deleted"]["level"],
            self.messages["account_deleted"]["text"].format(**{
                "expunge_hours": settings.ACCOUNT_DELETION_EXPUNGE_HOURS,
            })
        )
        return redirect(self.get_redirect_url())

    def get_context_data(self, **kwargs):
        ctx = super(DeleteView, self).get_context_data(**kwargs)
        ctx.update(kwargs)
        ctx["ACCOUNT_DELETION_EXPUNGE_HOURS"] = settings.ACCOUNT_DELETION_EXPUNGE_HOURS
        return ctx
