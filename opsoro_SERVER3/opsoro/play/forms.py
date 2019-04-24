from datetime import date, datetime

from captcha.fields import ReCaptchaField
from django import forms
from django.utils.translation import ugettext_lazy as _

from .models import *


class FeedbackForm(forms.Form):
    CATEGORY = (
        ('B', 'Bug'),
        ('I', 'Idea'),
        ('H', 'Help'),
    )
    text = forms.CharField(label=_("Tell us more!"), max_length=1000, required=True, widget=forms.Textarea())
    page = forms.CharField(max_length=200, required=False, widget=forms.HiddenInput())
    category = forms.CharField(max_length=1, required=False, widget=forms.HiddenInput())

    # captcha = ReCaptchaField()

    def clean(self):
        if not self.errors:

            if 'text' in self.cleaned_data:
                text = self.cleaned_data['text']
                # po.text = text
            else:
                raise forms.ValidationError(_("Invalid text."))

            if 'page' in self.cleaned_data:
                page = self.cleaned_data['page']
                # po.page = page

            if 'category' in self.cleaned_data:
                category = self.cleaned_data['category']
                # if category:
                #     po.category = str(category)

            # po.save()
            return self.cleaned_data
        raise forms.ValidationError(_("Invalid data."))
