
let indice = 0;
let div;
let imagenes = [
    "url('img/liminal_meeting_room.jpg')",
    "url('img/liminal_office_cubicles_1.jpg')",
    "url('img/liminal_meeting_room_2.jpg')",
    "url('img/liminal_office_cubicles_2.jpg')",
];
let posiciones = [
    "0% 55%",
    "100% 50%",
    "0% 57%",
    "100% 45%",
];

function cambiarFondo() {
    div.style.backgroundImage = imagenes[indice];
    div.style.backgroundPosition = posiciones[indice];
    indice = (indice + 1) % imagenes.length; // Cicla entre las imágenes
}

document.addEventListener('DOMContentLoaded', event => {
    div = document.getElementById("intro-section");
    cambiarFondo();
    setInterval(cambiarFondo, 5000);
})