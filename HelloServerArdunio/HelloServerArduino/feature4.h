String feature4Pagepart1 = F(R"=====(<!DOCTYPE html>
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
        function fetchEnergy() {
        fetch('/energyMeter') 
        .then(response => response.text()) 
        .then(simulatedPower => {
          console.log("Simulated Power:", simulatedPower); 
          document.getElementById("energyValue").innerText = simulatedPower; 
        })
        .catch(error => console.error('Error fetching power:', error)); // Handle errors
    }
    </script>

    window.onload = fetchEnergy;
</head>

<body>
    <h1>Power Load Measurement</h1>

    <p>Power Load Value: <span id="energyValue">Unknown</span></p>

</body>
</html>)=====");
