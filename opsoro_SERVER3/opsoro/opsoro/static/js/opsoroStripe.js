

var handler = StripeCheckout.configure({
  key: 'pk_test_pUFlEZUNSrMkBtK71E5kFEGV',
  image: 'https://stripe.com/img/documentation/checkout/marketplace.png',
  locale: 'auto',
  token: function(token) {
    // You can access the token ID with `token.id`.
    // Get the token ID to your server-side code for use.
  }
});

function buyProduct(id){

   var pid = '#product'+id;
   $('#OrderImg').attr('src',$(pid).find('img').attr('src'));
   $('#OrderName').text($(pid).find('h1').text());
   $('#OrderPrice').text($(pid).find('.pricing').text());
   $('#OrderSlogan').text($(pid).find('.feature').text());
   $('#id_product_id').val(id);
   $('#id_product_id').hide();
}
