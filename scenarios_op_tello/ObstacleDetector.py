import torch

# Télécharger le modèle et le sauvegarder
model = torch.hub.load("pytorch/vision:v0.10.0", "deeplabv3_resnet101", pretrained=True)
torch.save(model.state_dict(), "deeplabv3_resnet101.pth")
print("Modèle téléchargé et sauvegardé.")