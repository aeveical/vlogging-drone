const slides = [
    {
        img: "images/coffinparts.png",
        title: "Drone Design",
        text: "We decided on the shape of the drone early into the design process. We knew it would have to be big enough to house the Nvidia Jetson since we wanted the high processing power. After some sketches we decided on a quadcopter with a coffin shape as that design could balance out the center of mass and efficiently house all our electronics. The drones max weight is 2.4kg so we designed the drone chassis to pack each component inside with minimum extraneous space, which meant that wire management was difficult as we had little tolerance for wires lengths. The battery resides underneath the drone for weight balance and easy access since we need to recharge it frequently."
    },

    {
        img: "images/mounts.png",
        title: "Propller Hub",
        text: "Another important design choice was the propeller hub, we re-designed it a few times because the tolerances were too loose, the wires couldn't fit, and because the propeller hub sheared along the layer lines as the force applied from the propellers during an impact would shear the propeller hub. "
    },
    {
        img: "images/vents.png",
        title: "Cooling System",
        text: "With the Nvidia Jetson and the four ESC packed so closely in the front of the drone, the inside of the drone heats up quickly so we added cooling vents in the front and back of the drone."
    },
    {
        img: "images/yoloo.png",
        title: "Computer Vision",
        text: "For person detection we used You Only Look Once (YOLO) as it is the most well documented deep learning model for object detection. "
    },
    {
        img: "images/pixhawk.jpg",
        title: "Flight Controller",
        text: "We originally planned to use an Ardupilot APM2.8 for flight controller but decided to go with the more expensive Pixhawk 2.4.8 as it was able to handle more complicated instructions. "
    },
    {
        img: "images/livefeed.png",
        title: "Vlogging Aspect",
        text: "For vlogging, we need to be able to record video so we decided to have the camera write data to an SD card connected to the Nvidia Jetson and have a webserver hosted off the Jetson that shows live feed view of the camera when in autonomous mode. "
    }
];

let currentSlide = 0;

function updateSlide() {
    document.getElementById("sliderImg").src = slides[currentSlide].img;
    document.getElementById("sliderTitle").textContent = slides[currentSlide].title;
    document.getElementById("sliderText").textContent = slides[currentSlide].text;
}

function nextSlide() {
    currentSlide = (currentSlide + 1) % slides.length;
    updateSlide();
}

function prevSlide() {
    currentSlide = (currentSlide - 1 + slides.length) % slides.length;
    updateSlide();
}
