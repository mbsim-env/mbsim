var cgiPath="/cgi-bin/auth2.py";//MFMF

// indicate start of server commnication
function statusCommunicating() {
  // disable the buttons
  $("button").prop("disabled", true);
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
  }
  statusmsg.text(response.message);
  // enable the buttons
  $("button").prop("disabled", false);
  $("body").css("cursor", "default");
}

$(document).ready(function() {
// when the login button is clicked redirect to github auth page
  var loginWindow;
  $("#LOGINBUTTON").click(function() {
    statusCommunicating();
    loginWindow=window.open("https://github.com/login/oauth/authorize?client_id=2d12f6576e23c7ba04a4&redirect_uri="+
      window.location.origin+cgiPath+"/login&scope=read:org");
  })
  // and install a event listener to react on a successfull login on this page
  window.addEventListener("message", loginCallback, false);
  function loginCallback(event) {
    // do nothing for wrong origin
    if(event.origin !== window.location.origin)
      return;
    // close opened github login window and display the status message
    loginWindow.close();
    loginStatus();
    statusMessage({success: true, message: event.data})
  }

  // when the logout button is clicked
  $("#LOGOUTBUTTON").click(function() {
    statusCommunicating();
    // remove stored login from browser
    var loginname=localStorage['GITHUB_LOGIN_NAME'];
    localStorage.removeItem('GITHUB_LOGIN_NAME');
    localStorage.removeItem('GITHUB_LOGIN_ATHMAC');
    // and from server
    $.ajax({url: cgiPath+"/logout",
            dataType: "json", type: "POST", data: JSON.stringify({login: loginname})
          }).done(function(response) {
      loginStatus();
      statusMessage(response);
    });
  })

  // login status
  function loginStatus() {
    if(localStorage['GITHUB_LOGIN_NAME']) {
      $.ajax({url: cgiPath+"/getuser",
              dataType: "json", type: "POST", data: JSON.stringify({login: localStorage['GITHUB_LOGIN_NAME']})
            }).done(function(response) {
        if(!response.success)
          $('#LOGINUSER').text("Internal error: +"+response.message);
        else {
          $('#LOGINUSERIMG').attr("src", response.avatarurl);
          $('#LOGINUSER').text(response.username);
        }
      });
    }
    else {
      $('#LOGINUSER').text("Not logged in");
    }
  }
  loginStatus();
});
