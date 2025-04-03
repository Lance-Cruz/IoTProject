String feature3Pagepart1 = F(R"=====(<!DOCTYPE html>
<html>
<head>
    <title> SmartHaven Webpage </title>
    
    <style>
      body{
        background-color: #e3f2fd;
        text-align: center;
    }

    h1{
        font-size: xx-large;
        margin-bottom: 5px;
        color: #0eb917;
        text-shadow: 1px 1px 1px black;
        text-align: center;
    }

    p{
        font-size: larger;
    }

    .container{
        justify-content: space-evenly;
        display: flex;
        flex-flow: row wrap;
    }

    .feature {
        border: 2px solid black;
        margin-bottom: 10px;
        width: 300px;
        background-color: white;
    }

    .button {
        background-color: #0eb917;
        border: none;
        color: white;
        padding: 15px 32px;
        cursor: pointer;
    }
    </style>

    <script>
        function getPIR() {
        fetch('/getPIR')
        .then(response => response.text())
        .then(status => {
          console.log("Motion Sensor status:", status);
          document.getElementById("pirStatus").innerText = status;
        })
        .catch(error => console.error('Error fetch motion sensor status:', error));
    }

    setInterval(getPIR, 2000);

    window.onload = getPIR;
    </script>
</head>

<body>
    <h1>Front Door Security</h1>

    <iframe width="560" height="315" src="http://192.168.148.164:81/stream" frameborder="0" alt="camera stream" allowfullscreen></iframe>

    <p>Motion Sensor status: <span id="pirStatus">Unknown</span></p>

    <button class="button">Click to check camera</button>
</body>
</html>)=====");
