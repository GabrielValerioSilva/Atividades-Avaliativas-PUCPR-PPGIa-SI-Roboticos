<p align="center">
  <img src="assets/logo-pucpr.png" alt="Logo da PUCPR" width="220">
</p>   

<h1 align="center">Atividades — Sistemas Robóticos</h1>

<p align="center">PUCPR · Pontifícia Universidade Católica do Paraná</p>

## Sobre

Repositório das atividades da disciplina de Sistemas Robóticos.

## Atividade 1 - Fundamentos de ROS 2 e Turtlesim
A atividade trata-se de obvserver a comunicação entre o nós do turlesim ,utilizando conceitos aprendidos como o Publisher,Subscriber que estão dentro do  Topic.

lembrando que nós(popularmente chamado em inglês como: Node) é um programa independente que executa uma tarefa específica no robô, como ler um sensor ou acionar um motor.
Cada nó tem uma única responsabilidade o que lhe confere baixo acoplamento e dependências.


Um Tópico (Topic) no ROS 2 é um canal de comunicação assíncrono baseado no modelo publicar/assinar, usado para transmitir informações continuamente de um nó para outro no robô


O Tópico é o Mural: É o local com um nome fixo e específico, onde a informação fica disponível. O mural em si não cria nem lê dados, ele serve apenas como o canal.

O Publisher (Publicador) é a Instituição que lança o Edital: É o nó responsável por gerar os dados (ex: o nó da câmera) e pregar a informação no mural.

O Subscriber (Inscrito) é o Candidato atento ao Mural: É o nó interessado naquele assunto (ex: o nó de IA). Ele fica inscrito no tópico e "lê" a informação assim que uma nova publicação é colada.É o nó que assina

A Mensagem é o Papel/Documento do Edital: É o texto com os campos preenchidos de acordo com um formulário oficial  contendo os dados concretos
em outras palavras: é a estrutura de dados;funciona como uma struct em C , agrupando campos com tipos bem definidos


Service é a comunicação síncrono (Request/Response) usado para interações pontuais do tipo "pergunta e resposta" ou "comando e confirmação" entre dois nós
ex: client quer saber a localização atual do robô
server: ele é quem calciula e ja responde devolvendo a posição


Action (Ação) é o mecanismo de comunicação assíncrono projetado para comandos e tarefas de longa duração que levam tempo para serem concluídos

Parâmetros: Fazem parte da programação dinâmica,altera o comportamento de um nó sem recompilação

# Atividade 5
Objetivo: 
Realizar a implementação prática de um sistema de detecção de objetos em visão computacional por meio do retreinamento (fine-tuning) da arquitetura YOLO26 para reconhecer uma classe personalizada inexistente no dataset COCO padrão: um mini-skate de dedo (Tech Deck, rotulado como skate-de-dedo). O trabalho abrange desde a coleta e anotação de imagens próprias até o treinamento com ajuste dinâmico de taxa de aprendizado, avaliação por métricas quantitativas e inferência em vídeo.

Desenvolvimento:
Arquitetura do Sistema
O pipeline foi estruturado de forma modular em 6 células no ambiente Google Colab com aceleração por GPU (NVIDIA T4), dividido em quatro etapas principais:

Ingestão e Padronização de Dados: Extração automatizada do conjunto de dados exportado do Roboflow (31 imagens reais capturadas em diferentes ângulos e iluminações), convertendo anotações do formato COCO JSON para o formato nativo YOLO (coordenadas normalizadas).

Transfer Learning (Fine-Tuning): Utilização dos pesos pré-treinados da rede yolo26s.pt (Small), descartando intencionalmente variantes ultraleves (Nano) para garantir maior capacidade de extração de características geométricas e texturas finas (lixa, shape, trucks e rodinhas).

Otimização e Regularização: Aplicação de Data Augmentation em tempo de execução (rotação de 
±20 
∘
 , variação de escala de 
50% e ajustes no espaço de cores HSV de saturação e brilho) combinada ao escalonador ReduceLROnPlateau do PyTorch, acoplado ao ciclo de treinamento via callback (on_fit_epoch_end), reduzindo a taxa de aprendizado pela metade sempre que a perda de validação estagna.

Pós-processamento e Inferência: Filtragem das predições em vídeo utilizando limiar de confiança (conf), limite máximo de detecções por quadro (max_det=1) e Non-Maximum Suppression (NMS) (iou=0.45) para eliminar caixas delimitadoras (bounding boxes) duplicadas ou sobrepostas sobre o mesmo objeto.


Algoritmos usados:
Non-Maximum Suppression (NMS): Algoritmo de pós-processamento que ordena as caixas candidatas pelo escore de confiança e suprime aquelas cuja interseção sobre união (IoU) com a caixa principal ultrapassa o limiar definido (0.45).

ReduceLROnPlateau: Estratégia adaptativa de otimização que monitora a soma das perdas de validação (val/box_loss + val/cls_loss + val/dfl_loss) e reduz o learning rate (factor=0.5, patience=6, min_lr=1e-6) para estabilizar a convergência em datasets compactos.

Gargalo na Rotulagem Automática e Formato de Exportação do Roboflow:

Problema: O recurso de Auto Label em nuvem do Roboflow apresentou lentidão na fila gratuita, e o arquivo exportado veio estruturado no padrão COCO JSON em vez do arquivo data.yaml esperado nativamente pelo YOLO.

Solução: Realização da rotulagem manual precisa das 31 imagens e implementação de um conversor automático em Python (preparar_dataset_31_fotos) que transforma as coordenadas COCO em rótulos YOLO normalizados.

Falsos Positivos no Fundo e Caixas Delimitadoras Duplicadas:

Problema: Nos testes iniciais utilizando variantes Nano (yolo11n/yolo26n) e limiares padrão de supressão, o detector gerava múltiplas caixas fragmentadas sobre o mesmo skate (separando shape e rodinhas) e falsos positivos em texturas do ambiente (cortina e borda do mousepad).

Solução: Substituição pela arquitetura yolo26s.pt (Small), ajuste do limiar de IoU do NMS (iou=0.45) e restrição de detecções máximas por quadro (max_det=1).

Divergência de Escala (Enquadramento) entre Treino e Vídeo de Teste:

Problema: As 31 imagens do dataset foram capturadas em close-up (com o skate ocupando ~80% da altura da imagem), enquanto o primeiro vídeo de teste foi gravado com a câmera distante (640x384), fazendo com que o modelo não detectasse o objeto pequeno com limiar conf=0.35.

Solução: Ajuste do limiar de confiança na inferência (conf=0.10) e gravação do vídeo de demonstração em enquadramento próximo (close-up), alinhando a distribuição espacial da inferência aos dados de treinamento.

Oscilação das Perdas em Dataset Compacto:

Problema: Com apenas 31 imagens, taxas de aprendizado fixas ou decaimento cosseno padrão provocavam picos de instabilidade na perda de validação.

Solução: Implementação do callback com ReduceLROnPlateau, reduzindo o learning rate dinamicamente diante de platôs na perda de validação e permitindo convergência suave nas épocas finais.

## Atividades

- [Atividade 1](Atv1/)
- [Atividade 2](Atv2/)
- [Atividade 3](Atv3/)
- [Atividade 4](Atv4/)
- [Atividade 5](Atv5/)
- [Atividade 6](Atv6/)