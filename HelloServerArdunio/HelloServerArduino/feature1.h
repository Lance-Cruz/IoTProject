String feature1Pagepart1 = F(R"=====(<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
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
        function fetchTemperature() {
        fetch('/temperature') // Call the ESP32 server
        .then(response => response.text()) // Convert response to text
        .then(temp => {
          console.log("Temperature:", temp); // Debugging output in console
          document.getElementById("tempValue").innerText = temp; // Update webpage dynamically
        })
        .catch(error => console.error('Error fetching temperature:', error)); // Handle errors
    }

        function toggleFan() {
        fetch('/toggleFan')
        .then(response => response.text()) 
        .then(status => {
          console.log("Fan Status:", status); 
          document.getElementById("fanStatus").innerText = status; 
        })
        .catch(error => console.error('Error toggling Fan:', error));
    }


    // Fetch temperature every 2 seconds
    setInterval(fetchTemperature, 2000);

    // Fetch immediately on page load
    window.onload = fetchTemperature;
    </script>
</head>

<body>
    <h1>Humidty and Temperature</h1>

    <iframe width="450" height="260" style="border: 1px solid #cccccc;" src="https://thingspeak.com/channels/2713002/charts/1?bgcolor=%23ffffff&color=%23d62020&dynamic=true&results=60&title=Temperature&type=line&xaxis=Time&yaxis=Temp+C"></iframe>

    <iframe width="450" height="260" style="border: 1px solid #cccccc;" src="https://thingspeak.com/channels/2713002/charts/2?bgcolor=%23ffffff&color=%23d62020&dynamic=true&results=60&title=Humidity&type=line&xaxis=Time&yaxis=%25+Humidity"></iframe>

    <iframe width="450" height="260" style="border: 1px solid #cccccc;" src="https://thingspeak.com/channels/2713002/widgets/1022339"></iframe>

    <p>
        Current values below
    </p>

    <p>Temperature: <span id="tempValue">Loading...</span> °C</p>

    <p>Fan Status: <span id="fanStatus">Unknown</span></p>

    <button class="button" onclick="toggleFan()">Manually turn on Fan</button>
</body>
</html>)=====");
