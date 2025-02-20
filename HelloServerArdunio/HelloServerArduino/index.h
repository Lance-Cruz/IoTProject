String indexPagepart1 = F(R"=====(<!DOCTYPE html>
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
</head>
<body>
    <h1>SmartHaven</h1>

    <p>Control and monitor your home with IoT devices</p>
    
    <div class="container">
        <div class="feature">
            <p><a href="feature1.html"> Feature 1 </p></a>
        </div>

        <div class="feature">
            <p><a href="feature2.html"> Feature 2</p></a>
        </div>

        <div class="feature">
            <p><a href="feature3.html"> Feature 3</p></a>
        </div>

        <div class="feature">
            <p><a href="feature4.html"> Feature 4</p></a>
        </div>
    </div>  

</body>
</html>)=====");
