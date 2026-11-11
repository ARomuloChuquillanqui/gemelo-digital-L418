#!/usr/bin/env bash
# Ejecutar UNA vez en la VM, dentro de servidor_iot_prod:  bash preparar.sh
set -euo pipefail
cd "$(dirname "$0")"
set -a; source .env; set +a

if [[ "$DOMINIO" == TU-NOMBRE* ]]; then
  echo "ERROR: completa DOMINIO y las contraseñas en .env"; exit 1
fi

# 1) Certificados TLS para MQTT: CA propia válida por 10 años.
#    ca/ NO se monta en ningún contenedor; ca.crt es lo que se copia al gateway.
mkdir -p ca mosquitto/certs
if [ ! -f ca/ca.crt ]; then
  echo ">> Generando certificados TLS para $DOMINIO"
  openssl req -x509 -newkey rsa:2048 -nodes -days 3650 \
    -keyout ca/ca.key -out ca/ca.crt -subj "/CN=Gemelo L418 CA"
  openssl req -newkey rsa:2048 -nodes \
    -keyout mosquitto/certs/server.key -out /tmp/server.csr -subj "/CN=$DOMINIO"
  printf "subjectAltName=DNS:%s\n" "$DOMINIO" > /tmp/san.ext
  openssl x509 -req -in /tmp/server.csr -CA ca/ca.crt -CAkey ca/ca.key \
    -CAcreateserial -out mosquitto/certs/server.crt -days 3650 -extfile /tmp/san.ext
  rm -f /tmp/server.csr /tmp/san.ext
  chmod 600 ca/ca.key
else
  echo ">> Certificados ya existen, no se regeneran"
fi

# 2) Usuarios MQTT (se regeneran cada vez que ejecutas el script)
echo ">> Creando usuarios MQTT"
docker run --rm -v "$PWD/mosquitto/config:/mosquitto/config" eclipse-mosquitto:2 sh -c "
  mosquitto_passwd -b -c /mosquitto/config/passwd telegraf '$MQTT_TELEGRAF_PASS' &&
  mosquitto_passwd -b /mosquitto/config/passwd grupo1 '$MQTT_GRUPO1_PASS'"

# 3) Permisos: Mosquitto corre con el usuario UID 1883
sudo chown -R 1883:1883 mosquitto/config mosquitto/certs
sudo chmod 600 mosquitto/config/passwd mosquitto/config/acl mosquitto/certs/server.key

echo ""
echo ">> Listo. Ahora ejecuta:  docker compose up -d"
echo ">> Certificado para el gateway (cópialo completo en gateway.ino):"
cat ca/ca.crt
