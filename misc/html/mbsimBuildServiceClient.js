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
  var loginWindow;
  $("#LOGINBUTTON").click(function() {
    statusCommunicating();
    loginWindow=window.open("https://github.com/login/oauth/authorize?client_id=987997eb60fc086e9707&scope=read:org,public_repo,user:email");
  })
  // and install a event listener to react on a successfull login on this page
  window.addEventListener("message", loginCallback, false);
  function loginCallback(event) {
    // do nothing for wrong origin
    if(event.origin!=="https://www.mbsim-env.de")
      return;
    // close opened github login window and display the status message
    loginWindow.close();
    loginStatus();
    statusMessage({success: true, message: event.data})
  }

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
      }
      else {
        $('#LOGINUSER').text(response.username);
        $('#LOGINAVATAR').attr("src", response.avatar_url);
      }
    });
  }
  loginStatus();
});
