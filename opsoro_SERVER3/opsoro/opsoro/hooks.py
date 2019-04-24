import hashlib
import random

from account.conf import settings
from django.core.mail import EmailMultiAlternatives
from django.template import Context
from django.template.loader import get_template, render_to_string


class AccountHookSet(object):

    # subject = "I am an HTML email"
    # to = ['buddy@buddylindsey.com']
    # from_email = 'test@example.com'
    #
    # ctx = {
    #     'user': 'buddy',
    #     'purchase': 'Books'
    # }
    #
    # message = get_template('main/email/email.html').render(Context(ctx))
    # msg = EmailMessage(subject, message, to=to, from_email=from_email)
    # msg.content_subtype = 'html'
    # msg.send()
    #

    def send_invitation_email(self, to, ctx):
        print('send_invitation_email')
        subject = render_to_string("account/email/invite_user_subject.txt", ctx)
        # message = render_to_string("account/email/invite_user.txt", ctx)
        # message = get_template('account/email/invite_user.html').render(Context(ctx))
        # send_mail(subject, message, settings.DEFAULT_FROM_EMAIL, to)

        text_content = render_to_string("account/email/invite_user.txt", ctx)
        html_content = get_template('account/email/invite_user.html').render(Context(ctx))
        msg = EmailMultiAlternatives(subject, text_content, settings.DEFAULT_FROM_EMAIL, to)
        msg.attach_alternative(html_content, "text/html")
        msg.send()

    def send_confirmation_email(self, to, ctx):
        print('send_confirmation_email')
        subject = render_to_string("account/email/email_confirmation_subject.txt", ctx)
        subject = "".join(subject.splitlines())  # remove superfluous line breaks
        # message = render_to_string("account/email/email_confirmation_message.txt", ctx)
        # message = get_template('account/email/email_confirmation_message.html').render(Context(ctx))
        # # send_mail(subject, message, settings.DEFAULT_FROM_EMAIL, to)
        # msg = EmailMessage(subject, message, to=to, from_email=settings.DEFAULT_FROM_EMAIL)
        # msg.content_subtype = 'html'
        # msg.send()

        text_content = render_to_string("account/email/email_confirmation_message.txt", ctx)
        html_content = get_template('account/email/email_confirmation_message.html').render(Context(ctx))
        msg = EmailMultiAlternatives(subject, text_content, settings.DEFAULT_FROM_EMAIL, to)
        msg.attach_alternative(html_content, "text/html")
        msg.send()

    def send_password_change_email(self, to, ctx):
        print('send_password_change_email')
        subject = render_to_string("account/email/password_change_subject.txt", ctx)
        subject = "".join(subject.splitlines())
        # message = render_to_string("account/email/password_change.txt", ctx)
        # message = get_template('account/email/password_change.html').render(Context(ctx))
        # # send_mail(subject, message, settings.DEFAULT_FROM_EMAIL, to)
        # msg = EmailMessage(subject, message, to=to, from_email=settings.DEFAULT_FROM_EMAIL)
        # msg.content_subtype = 'html'
        # msg.send()

        text_content = render_to_string("account/email/password_change.txt", ctx)
        html_content = get_template('account/email/password_change.html').render(Context(ctx))
        msg = EmailMultiAlternatives(subject, text_content, settings.DEFAULT_FROM_EMAIL, to)
        msg.attach_alternative(html_content, "text/html")
        msg.send()

    def send_password_reset_email(self, to, ctx):
        print('send_password_reset_email')
        subject = render_to_string("account/email/password_reset_subject.txt", ctx)
        subject = "".join(subject.splitlines())
        # message = render_to_string("account/email/password_reset.txt", ctx)
        # message = get_template('account/email/password_reset.html').render(Context(ctx))
        # # send_mail(subject, message, settings.DEFAULT_FROM_EMAIL, to)
        # msg = EmailMessage(subject, message, to=to, from_email=settings.DEFAULT_FROM_EMAIL)
        # msg.content_subtype = 'html'
        # msg.send()

        text_content = render_to_string("account/email/password_reset.txt", ctx)
        html_content = get_template('account/email/password_reset.html').render(Context(ctx))
        msg = EmailMultiAlternatives(subject, text_content, settings.DEFAULT_FROM_EMAIL, to)
        msg.attach_alternative(html_content, "text/html")
        msg.send()

    def send_preorder_confirmation_email(self, to, ctx):
        print('send_password_reset_email')
        subject = render_to_string("account/email/preorder_confirmation_subject.txt", ctx)
        subject = "".join(subject.splitlines())
        # message = render_to_string("account/email/password_reset.txt", ctx)
        # message = get_template('account/email/preorder_confirmation.html').render(Context(ctx))
        # # send_mail(subject, message, settings.DEFAULT_FROM_EMAIL, to)
        # msg = EmailMessage(subject, message, to=to, from_email=settings.DEFAULT_FROM_EMAIL)
        # msg.content_subtype = 'html'
        # msg.send()

        text_content = render_to_string("account/email/preorder_confirmation.txt", ctx)
        html_content = get_template('account/email/preorder_confirmation.html').render(Context(ctx))
        msg = EmailMultiAlternatives(subject, text_content, settings.DEFAULT_FROM_EMAIL, to)
        msg.attach_alternative(html_content, "text/html")
        msg.send()

    def generate_random_token(self, extra=None, hash_func=hashlib.sha256):
        if extra is None:
            extra = []
        bits = extra + [str(random.SystemRandom().getrandbits(512))]
        return hash_func("".join(bits).encode("utf-8")).hexdigest()

    def generate_signup_code_token(self, email=None):
        extra = []
        if email:
            extra.append(email)
        return self.generate_random_token(extra)

    def generate_email_confirmation_token(self, email):
        return self.generate_random_token([email])

    def get_user_credentials(self, form, identifier_field):
        return {
            "username": form.cleaned_data[identifier_field],
            "password": form.cleaned_data["password"],
        }

    def account_delete_mark(self, deletion):
        deletion.user.is_active = False
        deletion.user.save()

    def account_delete_expunge(self, deletion):
        deletion.user.delete()
