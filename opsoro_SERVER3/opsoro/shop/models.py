from __future__ import unicode_literals

from datetime import datetime

import stripe
from account.hooks import hookset
from django.contrib.auth.models import User
from django.contrib.postgres.fields import ArrayField
from django.db import models
from stdimage.models import StdImageField

from .classes import *
from opsoro import settings


class Product(models.Model):
    id = models.AutoField(primary_key=True)
    stripe_id = models.CharField(max_length=50, null=True, blank=True)
    active = models.BooleanField(default=True)
    name = models.CharField(max_length=50)
    slogan = models.CharField(max_length=150, null=True, blank=True)
    description = models.CharField(max_length=500, null=True, blank=True)
    price = models.DecimalField(max_digits=8, decimal_places=2, null=True, default=0)
    shipping_cost = models.DecimalField(max_digits=6, decimal_places=2, null=True, default=0)
    stock = models.IntegerField(default='0', null=True)
    date = models.DateTimeField(auto_now_add=True, null=True, blank=True)

    def __str__(self):
        return self.name

    def save(self, *args, **kwargs):
        # if not self.stripe_id:
        #     prod = self.createStripeProd()
        #     self.stripe_id = prod.id
        # else:
        #     self.updateStripeProd()

        super(Product, self).save(*args, **kwargs)

    def createStripeProd(self):
        stripe.api_key = settings.STRIPE_API_KEY
        self.stripe = stripe

        prod = stripe.Product.create(
            name=self.name,
            active=self.active,
        )
        sku = stripe.SKU.create(
            product=prod.id,
            price=int(self.price * 100),
            currency="eur",
            inventory={
                "type": "infinite",
            }
        )
        return prod

    def updateStripeProd(self):
        stripe.api_key = settings.STRIPE_API_KEY
        self.stripe = stripe

        prod = stripe.Product.retrieve(self.stripe_id)
        prod.name = self.name
        prod.active = self.active
        sku = stripe.SKU.retrieve(prod.skus.data[0].id)
        sku.price = int(self.price * 100)
        sku.save()
        prod.save()

        return prod

    def substract_from_stock(self, amount):
        self.stock = self.stock - amount
        super(Product, self).save()

    def activate(self):
        self.active = True
        self.save()

    def deactivate(self):
        self.active = False
        self.save()


class ProductImage(models.Model):
    product = models.ForeignKey(Product, related_name='images', on_delete=models.CASCADE,)
    image = StdImageField(upload_to="products/", null=True, blank=True, variations={
        'medium': (900, 600),
        'thumbnail': (100, 100, True),
        'preload': (27, 17),
    })
    alt = models.CharField(max_length=50, blank=True)

    def __str__(self):
        return self.product.name


class ProductSpec(models.Model):
    product = models.ForeignKey(Product, related_name="specs", on_delete=models.CASCADE)
    spec = models.CharField(max_length=150, null=True)

    def __str__(self):
        return self.product.name


class Order(models.Model):
    ORDER_STATUS = (
        ('o', 'Ordered'),
        ('p', 'Paid'),
        ('s', 'Shipped'),
        ('c', 'Completed'),
        ('w', 'Cancelled'),
        ('x', 'Problem'),
    )

    charge_id = models.CharField(max_length=32,  null=True)
    order_id = models.CharField(max_length=32,  null=True)
    user = models.ForeignKey(User, related_name="orders", null=True, on_delete=models.PROTECT)
    order_date = models.DateTimeField(auto_now_add=True, null=True, blank=True)
    payment_date = models.DateTimeField(null=True, blank=True)
    shipping_date = models.DateTimeField(null=True, blank=True)
    delivery_date = models.DateTimeField(null=True, blank=True)
    status = models.CharField(max_length=1, choices=ORDER_STATUS)
    tracking_number = models.CharField(max_length=50,  default="Not available", null=True, blank=True)

    def __init__(self, *args, **kwargs):
        super(Order, self).__init__(*args, **kwargs)

        # stripe.api_key = settings.STRIPE_API_KEY
        # self.stripe = stripe

    def __str__(self):
        return str(self.order_date) + "  " + str(self.user)

    def save(self):
        #
        # old = Order.objects.get(pk=self.pk)
        # if old.status != self.status:
        #     sl = StatusLog()
        #     sl.create(self, self.get_status_display())

        super(Order, self).save()

    def buy_one(self, card, shipping, product):
        user = shipping.user

        if not user.profile.stripe_id:
            self.createCust(user)

        order = self.order_one(shipping, product)
        charge = self.pay(order, card, user)

        self.user = user
        self.save()

    def buy_all(self, card, shipping, orderlines):
        user = shipping.user

        if not user.profile.stripe_id:
            self.createCust(user)

        order = self.order_all(shipping, orderlines)
        charge = self.pay(order, card, user)

        self.user = user
        self.save()

    def createCust(self, user):
        response = self.stripe.Customer.create(
            email=user.email,
            description=user.first_name + user.last_name,
            shipping={
                "name": user.first_name + user.last_name,
                "address": {
                    "line1": user.profile.address,
                    "city": user.profile.city,
                    "country": user.profile.country,
                    "postal_code": user.profile.postal
                }}
        )

        print(response)
        user.profile.stripe_id = response.id
        user.save()

        return response

    def order_one(self, shipping, product):
        if self.order_id:
            return False, Exception(message="Already ordered.")

        cust = stripe.Customer.retrieve(shipping.user.profile.stripe_id)
        pro = stripe.Product.retrieve(product.stripe_id)

        response = self.stripe.Order.create(
            currency='eur',
            customer=cust.id,
            items=[
                {
                    "amount": int(product.price * 100),
                    "type": 'sku',
                    "parent": pro.skus.data[0].id
                }
            ],
            shipping={
                "name": shipping.name,
                "address": {
                    "line1": shipping.address,
                    "city": shipping.city,
                    "country": shipping.country,
                    "postal_code": shipping.postal,
                },
            },
        )

        self.order_id = response.id
        self.status = 'o'
        self.save()
        product.substract_from_stock(1)
        ol = OrderLine()
        ol.product = product
        ol.order = self
        ol.amount = 1
        ol.save()

        # mail sturen naar gebruiker

        return response

    def order_all(self, shipping, orderlines):
        if self.order_id:
            return False, Exception(message="Already ordered.")

        cust = stripe.Customer.retrieve(shipping.user.profile.stripe_id)

        items = []
        for ol in orderlines:
            p = stripe.Product.retrieve(ol.product.stripe_id)
            items.append({"amount": int(ol.product.price * 100), "quantity": ol.amount, "type": "sku", "parent": p.skus.data[0].id})

        response = self.stripe.Order.create(
            currency='eur',
            customer=cust.id,
            items=items,
            shipping={
                     "name": shipping.name,
                "address": {
                    "line1": shipping.address,
                    "city": shipping.city,
                    "country": shipping.country,
                    "postal_code": shipping.postal
                },
            },
        )

        self.order_id = response.id
        self.status = 'o'
        self.save()

        for ol in orderlines:
            o = OrderLine()
            o.product = ol.product
            ol.product.substract_from_stock(ol.amount)
            o.order = self
            o.amount = ol.amount
            o.save()

        # mail sturen naar gebruiker

        return response

    def pay(self, order, card, user):
        response = order.pay(
            source={
                "exp_month": card.exp_month,
                "exp_year": card.exp_year,
                "number": card.number,
                "object": "card",
                "cvc": card.cvc},
            email=user.email)

        self.charge_id = response.charge
        self.status = 'p'
        self.payment_date = datetime.now()

        # mail sturen naar gebruiker

        return response

    def ship(self):

        self.status = 's'
        self.shipping_date = datetime.now()
        order = stripe.Order.retrieve(self.order_id)
        order.status = 'fulfilled'
        order.save()
        self.save()

        # mail sturen naar gebruiker

    def received(self):
        self.status = 'c'
        self.delivery_date = datetime.now()
        self.save()

    def problem(self):
        self.status = 'x'
        self.save()


class OrderLine(models.Model):
    amount = models.IntegerField(default='1', null=True)
    order = models.ForeignKey(Order, related_name='orderlines', on_delete=models.CASCADE,)
    product = models.ForeignKey(Product, related_name='orderlines', on_delete=models.CASCADE)

    def create(self, order, product, amount):
        self.order = order
        self.product = product
        self.amount = amount
        self.save()


class DisputeMessage(models.Model):
    order = models.ForeignKey(Order, related_name='dispute_messages', on_delete=models.CASCADE,)
    user = models.ForeignKey(User, related_name='dispute_messages', on_delete=models.CASCADE)
    message = models.CharField(max_length=500, null=True, blank=True)
    message_date = models.DateTimeField(auto_now_add=True, null=True, blank=True)

    def create(self, order, message, user):

        self.order = order
        self.user = user
        self.message = message
        self.message_date = datetime.now()
        self.save()
        order.problem()


class StatusLog(models.Model):
    order = models.ForeignKey(Order, related_name='status_logs', on_delete=models.CASCADE,)
    status = models.CharField(max_length=500, blank=True,  null=True)
    date = models.DateTimeField(auto_now_add=True, null=True, blank=True)

    def create(self, order, status):
        self.order = order
        self.status = status
        self.date = datetime.now()
        self.save()


class PreOrder(models.Model):
    email = models.EmailField(max_length=100)
    name = models.CharField(max_length=150, blank=True, null=True)
    # extra = models.CharField(max_length=150, blank=True, null=True)
    product = models.ForeignKey(Product, related_name='PreOrders', null=True, blank=True, on_delete=models.CASCADE)
    date = models.DateTimeField(auto_now_add=True, null=True, blank=True)

    def __str__(self):
        if self.product:
            return self.product.name + ' : ' + self.email
        return self.email

    def save(self):
        if not PreOrder.objects.filter(email=self.email, product=self.product):
            super(PreOrder, self).save()
            # if self.product:
            #     if not self.product.active:
            #         return
            ctx = {
                "username": self.name,
            }
            hookset.send_preorder_confirmation_email([self.email], ctx)


class NewsLetter(models.Model):
    email = models.EmailField(max_length=100, unique=True)
    name = models.CharField(max_length=150, blank=True, null=True)
    date = models.DateTimeField(auto_now_add=True, null=True, blank=True)

    def __str__(self):
        return self.email

    def save(self):
        if not NewsLetter.objects.filter(email=self.email):
            super(NewsLetter, self).save()
