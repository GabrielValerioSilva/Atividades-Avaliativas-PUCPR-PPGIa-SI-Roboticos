<p align="center">
  <img src="assets/logo-pucpr.png" alt="Logo da PUCPR" width="220">
</p>   

<h1 align="center">Atividades — Sistemas Robóticos</h1>

<p align="center">PUCPR · Pontifícia Universidade Católica do Paraná</p>

## Sobre

Repositório das atividades da disciplina de Sistemas Robóticos.

Atividade 1 - Fundamentos de ROS 2 e Turtlesim
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

## Atividades

- [Atividade 1](Atv1/)
- [Atividade 2](Atv2/)
- [Atividade 3](Atv3/)
- [Atividade 4](Atv4/)
- [Atividade 5](Atv5/)
- [Atividade 6](Atv6/)