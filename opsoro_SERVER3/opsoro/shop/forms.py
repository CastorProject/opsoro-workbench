from calendar import monthrange
from datetime import date, datetime

from django import forms
from django.utils.translation import ugettext_lazy as _

from .classes import *
from .models import *


class CreditCardField(forms.IntegerField):
    def clean(self, value):
        """
        Check if given CC number is valid and one of the card types we accept
        """
        if value and (len(value) < 13 or len(value) > 16):
            raise forms.ValidationError(_("Please enter in a valid credit card number."))
        return super(CreditCardField, self).clean(value)


class CCExpWidget(forms.MultiWidget):
    """ Widget containing two select boxes for selecting the month and year"""

    def decompress(self, value):
        return [value.month, value.year] if value else [None, None]

    def format_output(self, rendered_widgets):
        html = u' / '.join(rendered_widgets)
        return u'<span style="white-space: nowrap;">%s</span>' % html


class CCExpField(forms.MultiValueField):
    EXP_MONTH = [(x, x) for x in range(1, 13)]
    EXP_YEAR = [(x, x) for x in range(date.today().year,
                                       date.today().year + 15)]
    default_error_messages = {
        'invalid_month': _('Enter a valid month.'),
        'invalid_year': _('Enter a valid year.'),
    }

    def __init__(self, *args, **kwargs):
        errors = self.default_error_messages.copy()
        if 'error_messages' in kwargs:
            errors.update(kwargs['error_messages'])
        fields = (
            forms.ChoiceField(choices=self.EXP_MONTH,
                              error_messages={'invalid': errors['invalid_month']}),
            forms.ChoiceField(choices=self.EXP_YEAR,
                              error_messages={'invalid': errors['invalid_year']}),
        )
        super(CCExpField, self).__init__(fields, *args, **kwargs)
        self.widget = CCExpWidget(widgets=[fields[0].widget, fields[1].widget])

    def clean(self, value):
        exp = super(CCExpField, self).clean(value)
        if date.today() > exp:
            raise forms.ValidationError(_("The expiration date you entered is in the past."))
        return exp

    def compress(self, data_list):
        if data_list:
            if data_list[1] in forms.fields.EMPTY_VALUES:
                error = self.error_messages['invalid_year']
                raise forms.ValidationError(error)
            if data_list[0] in forms.fields.EMPTY_VALUES:
                error = self.error_messages['invalid_month']
                raise forms.ValidationError(error)
            year = int(data_list[1])
            month = int(data_list[0])
            # find last day of the month
            day = monthrange(year, month)[1]
            return date(year, month, day)
        return None


class OrderPaymentForm(forms.Form):
    product_id = forms.IntegerField()
    number = CreditCardField(required=True, label=_("Card Number"), widget=forms.TextInput(attrs={'placeholder': _('Card Number')}))
    cvc = forms.IntegerField(required=True, label=_("CCV Number"), max_value=9999,
                             widget=forms.TextInput(attrs={'size': '4', 'placeholder': _('CVC')}))
    expiration = CCExpField(required=True, label=_("Expiration"))
    first_name = forms.CharField(label=_("First Name"), widget=forms.TextInput(attrs={'placeholder': _('First Name')}))
    last_name = forms.CharField(label=_("Last Name"), widget=forms.TextInput(attrs={'placeholder': _('Last Name')}))
    address = forms.CharField(label=_("Address"), widget=forms.TextInput(attrs={'placeholder': _('Address')}))
    postal = forms.CharField(label=_("Postal Code"), widget=forms.TextInput(attrs={'placeholder': _('Postal Code')}))
    city = forms.CharField(label=_("City"), widget=forms.TextInput(attrs={'placeholder': _('City')}))
    country = forms.CharField(label=_("Country"), widget=forms.TextInput(attrs={'placeholder': _('Country')}))

    def __init__(self, user, *args, **kwargs):
        self.user = user
        super(OrderPaymentForm, self).__init__(*args, **kwargs)

    def clean(self):
        """
        The clean method will effectively charge the card and create a new
        Order instance. If it fails, it simply raises the error given from
        Stripe's library as a standard ValidationError for proper feedback.
        """
        cleaned = super(OrderPaymentForm, self).clean()

        print(self.errors)

        if not self.errors:
            product_id = self.cleaned_data["product_id"]
            product = Product.objects.get(pk=product_id)

            card = Card()
            card.number = self.cleaned_data["number"]
            card.exp_month = self.cleaned_data["expiration"].month
            card.exp_year = self.cleaned_data["expiration"].year
            card.cvc = self.cleaned_data["cvc"]

            shipping = ShippingAddress()
            shipping.user = self.user
            shipping.name = self.cleaned_data["first_name"] + "  " + self.cleaned_data["last_name"]
            shipping.address = self.cleaned_data["address"]
            shipping.postal = self.cleaned_data["postal"]
            shipping.city = self.cleaned_data["city"]
            shipping.country = self.cleaned_data["country"]

            order = Order()
            order.buy_one(card, shipping, product)

        return cleaned


class PreOrderForm(forms.Form):
    p_id = forms.IntegerField(required=False)
    name = forms.CharField(label=_("Name"), widget=forms.TextInput(attrs={'placeholder': _('Name')}), required=False)
    email = forms.EmailField(label=_("Email"), widget=forms.EmailInput(attrs={'placeholder': _('Email')}), required=True)
    news = forms.BooleanField(required=False, label=_('NewsLetter'))

    def clean(self):
        if not self.errors:
            po = PreOrder()
            if "p_id" in self.cleaned_data:
                product_id = self.cleaned_data["p_id"]
                if product_id:
                    product = Product.objects.get(pk=product_id)
                    po.product = product

            if 'email' in self.cleaned_data:
                email = self.cleaned_data['email']
                po.email = email

                if 'news' in self.cleaned_data:
                    if self.cleaned_data['news']:
                        nl = NewsLetter()
                        nl.email = self.cleaned_data['email']
                        nl.save()
            else:
                raise forms.ValidationError(_("Invalid email address."))

            if 'name' in self.cleaned_data:
                name = self.cleaned_data['name']
                if name:
                    po.name = name

            po.save()
            return self.cleaned_data
        raise forms.ValidationError(_("Invalid data."))
