
function detectmob() {

    if( navigator.userAgent.match(/Android/i)
    || navigator.userAgent.match(/webOS/i)
    || navigator.userAgent.match(/iPhone/i)
    || navigator.userAgent.match(/iPod/i)
    || navigator.userAgent.match(/BlackBerry/i)
    || navigator.userAgent.match(/Windows Phone/i)
    ){
        // If mobile, then we do all this
    }
    else {
        // If not mobile then do this
        document.getElementById("bgvid").innerHTML = '<source src="static/images/cover/heroloop.webm" type="video/webm; codecs=vp9,vorbis"><source src="static/images/cover/heroloop.mp4" type="video/mp4">';

        // Move image to the back if the video starts playing
        document.getElementById("bgvid").onplay = function() {
          $('#bgvid').css({'opacity': '1'});
          $('.hero-img').css('z-index', '0');
        };
    }
}

function preOrderProduct(id){
  var pid = '#product'+id;
  $('#PreOrderModalImg').attr('src', $(pid).find('img').last().attr('src'));

  $('#id_p_id').val(id);
  $("label[for='id_p_id']").hide();
  $('#id_p_id').hide();
}



function validateEmail(email) {
    var re = /^(([^<>()[\]\\.,;:\s @\ "]+(\.[^<>()[\]\\.,;:\s@\"]+)*)|(\".+\"))@((\[[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\])|(([a-zA-Z\-0-9]+\.)+[a-zA-Z]{2,}))$/;
    return re.test(email);
}

$(document).ready(function() {
  detectmob();

  $('.testimonials_slider').slick({
    dots: true,
    arrows: false,
    infinite: true,
    speed: 600,
    slidesToShow: 1,
    slidesToScroll: 1,
    autoplay: true,
    autoplaySpeed: 8000,
    adaptiveHeight: false,
    fade: true,
    cssEase: 'linear',
  });

  $("#preOrder_form, #preOrderOne_form").submit(function(event) {
    $.ajax({ // create an AJAX call...
      data: $(this).serialize(), // get the form data
      type: $(this).attr('method'), // GET or POST
      url: $(this).attr('action'), // the file to call
      success: function(data) { // on success..
        $('.reveal-order-content').html(data['message']); // update the DIV
      }
    });
    return false;
  });
});
