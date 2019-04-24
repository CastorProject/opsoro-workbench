from django.contrib import admin

from .models import *


# Register your models here.


class ProductImageInline(admin.TabularInline):
    model = ProductImage
    extra = 1


class ProductSpecInline(admin.TabularInline):
    model = ProductSpec
    extra = 1


class ProductAdmin(admin.ModelAdmin):

    def deactivate_product(modeladmin, request, queryset):
        for product in queryset:
            product.deactivate()

    def activate_product(modeladmin, request, queryset):
        for product in queryset:
            product.activate()

    list_filter = ('active',)
    list_display = ('name', 'slogan', 'description', 'price', 'stock')
    inlines = [ProductImageInline, ProductSpecInline]
    actions = [deactivate_product, activate_product]


class OrderLineInline(admin.TabularInline):
    model = OrderLine
    extra = 0


class DisputeMessageInline(admin.TabularInline):
    model = DisputeMessage
    extra = 0


class StatusLogInline(admin.TabularInline):
    model = StatusLog
    readonly_fields = ('status', 'date',)
    extra = 0


class OrderAdmin(admin.ModelAdmin):

    def ship_order(modeladmin, request, queryset):
        for order in queryset:
            order.ship()
    ship_order.short_description = "Mark selected orders as shipped"

    list_filter = ('status',)
    list_display = ('order_id', 'user', 'order_date', 'status')
    actions = [ship_order]
    inlines = [OrderLineInline, DisputeMessageInline, StatusLogInline]


class PreOrderAdmin(admin.ModelAdmin):
    list_filter = ('product',)
    list_display = ('email', 'product', 'name', 'date')


class NewsLetterAdmin(admin.ModelAdmin):
    list_display = ('email', 'name', 'date')


admin.site.register(Order, OrderAdmin)
admin.site.register(Product, ProductAdmin)
admin.site.register(PreOrder, PreOrderAdmin)
admin.site.register(NewsLetter, NewsLetterAdmin)
