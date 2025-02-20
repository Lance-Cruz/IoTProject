String feature2Pagepart1 = F(R"=====(<!DOCTYPE html>
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
        function toggleLED() {
        fetch('/toggleLED')
        .then(response => response.text())
        .then(status => {
          console.log("LED Status:", status);
          document.getElementById("ledStatus").innerText = status;
        })
        .catch(error => console.error('Error toggling LED:', error));
    }
    </script>
</head>

<body>
    <h1>Automatic Light Sensor</h1>

    <iframe width="450" height="260" style="border: 1px solid #cccccc;" src="https://thingspeak.com/channels/2713002/charts/3?bgcolor=%23ffffff&color=%23d62020&dynamic=true&results=60&type=line&update=15"></iframe>

    <iframe width="450" height="260" style="border: 1px solid #cccccc;" src="https://thingspeak.com/channels/2713002/widgets/1022337"></iframe>

    <p>
        Current values below
    </p>

    <p>LED Status: <span id="ledStatus">Unknown</span></p>

    <button class="button" onclick="toggleLED()">Manually turn on light</button>
</body>
</html>)=====");
