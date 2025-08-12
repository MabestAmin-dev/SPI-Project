// You can add your JavaScript functionality here
// For example, handling button click events
document.addEventListener("DOMContentLoaded", function () {
    const buttons = document.querySelectorAll(".action-button");

    buttons.forEach((button) => {
        button.addEventListener("click", function () {
            // Perform an action when a button is clicked
            // You can add your custom action here
            alert("Button clicked!");
        });
    });
});
