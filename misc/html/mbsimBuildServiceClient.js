var cgiPath="https://www.mbsim-env.de/cgi-bin/mbsimBuildServiceServer.py";

// indicate start of server commnication
function statusCommunicating() {
  // disable the buttons
  $("._DISABLEONCOMM").each(function() {
    $(this).prop("disabled", true);
  });
  $("body").css("cursor", "progress");
  // set status
  var statuspanel=$("#STATUSPANEL");
  var statusmsg=$("#STATUSMSG");
  statuspanel.removeClass("panel-success");
  statuspanel.removeClass("panel-danger");
  statuspanel.addClass("panel-info");
  statusmsg.text("Communicating with server, please wait. (reload page if hanging)");
}

// indicate end of server commnication and print result message in status element
function statusMessage(response) {
  // set status
  var statuspanel=$("#STATUSPANEL");
  var statusmsg=$("#STATUSMSG");
  if(response.success) {
    statuspanel.removeClass("panel-info");
    statuspanel.removeClass("panel-danger");
    statuspanel.addClass("panel-success");
  }
  else {
    statuspanel.removeClass("panel-info");
    statuspanel.removeClass("panel-success");
    statuspanel.addClass("panel-danger");
    $(document.body).animate({'scrollTop': $('#STATUSPANEL').offset().top}, 750);
  }
  statusmsg.text(response.message);
  // enable the buttons
  $("._DISABLEONCOMM").each(function() {
    $(this).prop("disabled", false);
  });
  $("body").css("cursor", "default");
}

$(document).ready(function() {
  // when the login button is clicked redirect to github auth page
  $("#LOGINBUTTON").click(function() {
    statusCommunicating();
    // save current scroll value and goto github
    sessionStorage.setItem('backFromLogin', $(document).scrollTop());
    window.location.href="https://github.com/login/oauth/authorize?client_id=987997eb60fc086e9707&scope=read:org,public_repo,user:email";
  })

  // when the logout button is clicked
  $("#LOGOUTBUTTON").click(function() {
    statusCommunicating();
    // from server
    $.ajax({url: cgiPath+"/logout", xhrFields: {withCredentials: true}, dataType: "json", type: "GET"}).done(function(response) {
      loginStatus();
      statusMessage(response);
      if(response.success)
        document.cookie="mbsimenvsessionid_js=dummy; Domain=www.mbsim-env.de; Path=/; Expires=Thu, 01 Jan 1970 00:00:00 UTC; Secure;"
    });
  })

  // login status
  function loginStatus() {
    $.ajax({url: cgiPath+"/getuser", xhrFields: {withCredentials: true}, dataType: "json", type: "GET"}).done(function(response) {
      if(!response.success) {
        $('#LOGINUSER').text("Internal error: "+response.message);
        $('#LOGINAVATAR').attr("src", "data:image/gif;base64,R0lGODlhAQABAAD/ACwAAAAAAQABAAACADs=");
        statusMessage({success: false, message: response.message})
      }
      else {
        if(response.username)
        {
          $('#LOGINUSER').text(response.username);
          statusMessage({success: true, message: "User "+response.username+" is logged in."})
        }
        else {
          $('#LOGINUSER').text("not logged in");
          statusMessage({success: true, message: "No user is currently logged in. "+response.message})
        }
        $('#LOGINAVATAR').attr("src", response.avatar_url);
      }
    });
  }
  loginStatus();
  // if we are back from github login, restore scroll value and remove the storage always
  if(sessionStorage.getItem('backFromLogin')) {
    $(document).scrollTop(sessionStorage.getItem("backFromLogin"));
  }
  sessionStorage.removeItem('backFromLogin');
});
