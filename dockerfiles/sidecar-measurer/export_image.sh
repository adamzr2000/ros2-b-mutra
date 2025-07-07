# Sur le serveur source
# Construire l'image si ce n'est pas déjà fait
docker-compose -f docker-compose-sidecar1.yml build sidecar_measurer

# Sauvegarder l'image dans un fichier tar
docker save -o sidecar_measurer.tar sidecar_measurer:latest

