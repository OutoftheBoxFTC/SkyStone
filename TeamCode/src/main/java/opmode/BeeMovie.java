package opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.HashMap;

import hardware.ReadData;
import state.LogicState;

@TeleOp(name = "Bee Movie")
public class BeeMovie extends BasicOpmode {
    private static final String[] script = new String[]{
            "Bee Movie\n"+
                    "According to all known laws\n"+
                    "of aviation,",
            "there is no way a bee\n"+
                    "should be able to fly.",
            "Its wings are too small to get\n"+
                    "its fat little body off the ground.",
            "The bee, of course, flies anyway",
            "because bees don't care\n"+
                    "what humans think is impossible.",
            "Yellow, black. Yellow, black.\n"+
                    "Yellow, black. Yellow, black.",
            "Ooh, black and yellow!\n"+
                    "Let's shake it up a little.",
            "Barry! Breakfast is ready!",
            "Ooming!",
            "Hang on a second.",
            "Hello?",
            "- Barry?\n"+
                    "- Adam?",
            "- Oan you believe this is happening?\n"+
                    "- I can't. I'll pick you up.",
            "Looking sharp.",
            "Use the stairs. Your father\n"+
                    "paid good money for those.",
            "Sorry. I'm excited.",
            "Here's the graduate.\n"+
                    "We're very proud of you, son.",
            "A perfect report card, all B's.",
            "Very proud.",
            "Ma! I got a thing going here.",
            "- You got lint on your fuzz.\n"+
                    "- Ow! That's me!",
            "- Wave to us! We'll be in row 118,000.\n"+
                    "- Bye!",
            "Barry, I told you, stop flying in the house!",
            "- Hey, Adam.\n"+
                    "- Hey, Barry.",
            "- Is that fuzz gel?\n"+
                    "- A little. Special day, graduation.",
            "Never thought I'd make it.",
            "Three days grade school, three days high school.",
            "Those were awkward.",
            "Three days college. I'm glad I took\n"+
                    "a day and hitchhiked around the hive.",
            "You did come back different.",
            "- Hi, Barry.\n"+
                    "- Artie, growing a mustache? Looks good.",
            "- Hear about Frankie?\n"+
                    "- Yeah.",
            "- You going to the funeral?\n"+
                    "- No, I'm not going.",
            "Everybody knows, sting someone, you die.",
            "Don't waste it on a squirrel.\n"+
                    "Such a hothead.",
            "I guess he could have\n"+
                    "just gotten out of the way.",
            "I love this incorporating\n"+
                    "an amusement park into our day.",
            "That's why we don't need vacations.",
            "Boy, quite a bit of pomp...\n"+
                    "under the circumstances.",
            "- Well, Adam, today we are men.\n"+
                    "- We are!",
            "- Bee-men.\n"+
                    "- Amen!",
            "Hallelujah!",
            "Students, faculty, distinguished bees,",
            "please welcome Dean Buzzwell.",
            "Welcome, New Hive Oity\n"+
                    "graduating class of...",
            "...9:15.",
            "That concludes our ceremonies.",
            "And begins your career\n"+
                    "at Honex Industries!",
            "Will we pick ourjob today?",
            "I heard it's just orientation.",
            "Heads up! Here we go.",
            "Keep your hands and antennas\n"+
                    "inside the tram at all times.",
            "- Wonder what it'll be like?\n"+
                    "- A little scary.",
            "Welcome to Honex, a division of Honesco",
            "and a part of the Hexagon Group.",
            "This is it!",
            "Wow.",
            "Wow.",
            "We know that you, as a bee, have worked your whole life",
            "to get to the point where you\n"+
                    "can work for your whole life.",
            "Honey begins when our valiant Pollen\n"+
                    "Jocks bring the nectar to the hive.",
            "Our top-secret formula",
            "is automatically color-corrected, scent-adjusted and bubble-contoured",
            "into this soothing sweet syrup",
            "with its distinctive\n"+
                    "golden glow you know as...",
            "Honey!",
            "- That girl was hot.\n"+
                    "- She's my cousin!",
            "- She is?\n"+
                    "- Yes, we're all cousins.",
            "- Right. You're right.\n"+
                    "- At Honex, we constantly strive",
            "to improve every aspect\n"+
                    "of bee existence.",
            "These bees are stress-testing\n"+
                    "a new helmet technology.",
            "- What do you think he makes?\n"+
                    "- Not enough.",
            "Here we have our latest advancement, the Krelman.",
            "- What does that do?\n"+
                    "- Oatches that little strand of honey",
            "that hangs after you pour it.\n"+
                    "Saves us millions.",
            "Oan anyone work on the Krelman?",
            "Of course. Most bee jobs are\n"+
                    "small ones. But bees know",
            "that every small job, if it's done well, means a lot.",
            "But choose carefully",
            "because you'll stay in the job\n"+
                    "you pick for the rest of your life.",
            "The same job the rest of your life?\n"+
                    "I didn't know that.",
            "What's the difference?",
            "You'll be happy to know that bees, as a species, haven't had one day off",
            "in 27 million years.",
            "So you'll just work us to death?",
            "We'll sure try.",
            "Wow! That blew my mind!",
            "What's the difference?\n"+
                    "How can you say that?",
            "One job forever?\n"+
                    "That's an insane choice to have to make.",
            "I'm relieved. Now we only have\n"+
                    "to make one decision in life.",
            "But, Adam, how could they\n"+
                    "never have told us that?",
            "Why would you question anything?\n"+
                    "We're bees.",
            "We're the most perfectly\n"+
                    "functioning society on Earth.",
            "You ever think maybe things\n"+
                    "work a little too well here?",
            "Like what? Give me one example.",
            "I don't know. But you know\n"+
                    "what I'm talking about.",
            "Please clear the gate.\n"+
                    "Royal Nectar Force on approach.",
            "Wait a second. Oheck it out.",
            "- Hey, those are Pollen Jocks!\n"+
                    "- Wow.",
            "I've never seen them this close.",
            "They know what it's like\n"+
                    "outside the hive.",
            "Yeah, but some don't come back.",
            "- Hey, Jocks!\n"+
                    "- Hi, Jocks!",
            "You guys did great!",
            "You're monsters!\n"+
                    "You're sky freaks! I love it! I love it!",
            "- I wonder where they were.\n"+
                    "- I don't know.",
            "Their day's not planned.",
            "Outside the hive, flying who knows\n"+
                    "where, doing who knows what.",
            "You can'tjust decide to be a Pollen\n"+
                    "Jock. You have to be bred for that.",
            "Right.",
            "Look. That's more pollen\n"+
                    "than you and I will see in a lifetime.",
            "It's just a status symbol.\n"+
                    "Bees make too much of it.",
            "Perhaps. Unless you're wearing it\n"+
                    "and the ladies see you wearing it.",
            "Those ladies?\n"+
                    "Aren't they our cousins too?",
            "Distant. Distant.",
            "Look at these two.",
            "- Oouple of Hive Harrys.\n"+
                    "- Let's have fun with them.",
            "It must be dangerous\n"+
                    "being a Pollen Jock.",
            "Yeah. Once a bear pinned me\n"+
                    "against a mushroom!",
            "He had a paw on my throat, and with the other, he was slapping me!",
            "- Oh, my!\n"+
                    "- I never thought I'd knock him out.",
            "What were you doing during this?",
            "Trying to alert the authorities.",
            "I can autograph that.",
            "A little gusty out there today, wasn't it, comrades?",
            "Yeah. Gusty.",
            "We're hitting a sunflower patch\n"+
                    "six miles from here tomorrow.",
            "- Six miles, huh?\n"+
                    "- Barry!",
            "A puddle jump for us, but maybe you're not up for it.",
            "- Maybe I am.\n"+
                    "- You are not!",
            "We're going 0900 at J-Gate.",
            "What do you think, buzzy-boy?\n"+
                    "Are you bee enough?",
            "I might be. It all depends\n"+
                    "on what 0900 means.",
            "Hey, Honex!",
            "Dad, you surprised me.",
            "You decide what you're interested in?",
            "- Well, there's a lot of choices.\n"+
                    "- But you only get one.",
            "Do you ever get bored\n"+
                    "doing the same job every day?",
            "Son, let me tell you about stirring.",
            "You grab that stick, and you just\n"+
                    "move it around, and you stir it around.",
            "You get yourself into a rhythm.\n"+
                    "It's a beautiful thing.",
            "You know, Dad, the more I think about it,",
            "maybe the honey field\n"+
                    "just isn't right for me.",
            "You were thinking of what, making balloon animals?",
            "That's a bad job\n"+
                    "for a guy with a stinger.",
            "Janet, your son's not sure\n"+
                    "he wants to go into honey!",
            "- Barry, you are so funny sometimes.\n"+
                    "- I'm not trying to be funny.",
            "You're not funny! You're going\n"+
                    "into honey. Our son, the stirrer!",
            "- You're gonna be a stirrer?\n"+
                    "- No one's listening to me!",
            "Wait till you see the sticks I have.",
            "I could say anything right now.\n"+
                    "I'm gonna get an ant tattoo!",
            "Let's open some honey and celebrate!",
            "Maybe I'll pierce my thorax.\n"+
                    "Shave my antennae.",
            "Shack up with a grasshopper. Get\n"+
                    "a gold tooth and call everybody \"dawg!\"",
            "I'm so proud.",
            "- We're starting work today!\n"+
                    "- Today's the day.",
            "Oome on! All the good jobs\n"+
                    "will be gone.",
            "Yeah, right.",
            "Pollen counting, stunt bee, pouring, stirrer, front desk, hair removal...",
            "- Is it still available?\n"+
                    "- Hang on. Two left!",
            "One of them's yours! Oongratulations!\n"+
                    "Step to the side.",
            "- What'd you get?\n"+
                    "- Picking crud out. Stellar!",
            "Wow!",
            "Oouple of newbies?",
            "Yes, sir! Our first day! We are ready!",
            "Make your choice.",
            "- You want to go first?\n"+
                    "- No, you go.",
            "Oh, my. What's available?",
            "Restroom attendant's open, not for the reason you think.",
            "- Any chance of getting the Krelman?\n"+
                    "- Sure, you're on.",
            "I'm sorry, the Krelman just closed out.",
            "Wax monkey's always open.",
            "The Krelman opened up again.",
            "What happened?",
            "A bee died. Makes an opening. See?\n"+
                    "He's dead. Another dead one.",
            "Deady. Deadified. Two more dead.",
            "Dead from the neck up.\n"+
                    "Dead from the neck down. That's life!",
            "Oh, this is so hard!",
            "Heating, cooling, stunt bee, pourer, stirrer,",
            "humming, inspector number seven, lint coordinator, stripe supervisor,",
            "mite wrangler. Barry, what\n"+
                    "do you think I should... Barry?",
            "Barry!",
            "All right, we've got the sunflower patch\n"+
                    "in quadrant nine...",
            "What happened to you?\n"+
                    "Where are you?",
            "- I'm going out.\n"+
                    "- Out? Out where?",
            "- Out there.\n"+
                    "- Oh, no!",
            "I have to, before I go\n"+
                    "to work for the rest of my life.",
            "You're gonna die! You're crazy! Hello?",
            "Another call coming in.",
            "If anyone's feeling brave, there's a Korean deli on 83rd",
            "that gets their roses today.",
            "Hey, guys.",
            "- Look at that.\n"+
                    "- Isn't that the kid we saw yesterday?",
            "Hold it, son, flight deck's restricted.",
            "It's OK, Lou. We're gonna take him up.",
            "Really? Feeling lucky, are you?",
            "Sign here, here. Just initial that.",
            "- Thank you.\n"+
                    "- OK.",
            "You got a rain advisory today,",
            "and as you all know, bees cannot fly in rain.",
            "So be careful. As always, watch your brooms,",
            "hockey sticks, dogs, birds, bears and bats.",
            "Also, I got a couple of reports\n"+
                    "of root beer being poured on us.",
            "Murphy's in a home because of it, babbling like a cicada!",
            "- That's awful.\n"+
                    "- And a reminder for you rookies,",
            "bee law number one, absolutely no talking to humans!",
            "All right, launch positions!",
            "Buzz, buzz, buzz, buzz! Buzz, buzz, buzz, buzz! Buzz, buzz, buzz, buzz!",
            "Black and yellow!",
            "Hello!",
            "You ready for this, hot shot?",
            "Yeah. Yeah, bring it on.",
            "Wind, check.",
            "- Antennae, check.\n"+
                    "- Nectar pack, check.",
            "- Wings, check.\n"+
                    "- Stinger, check.",
            "Scared out of my shorts, check.",
            "OK, ladies,",
            "let's move it out!",
            "Pound those petunias, you striped stem-suckers!",
            "All of you, drain those flowers!",
            "Wow! I'm out!",
            "I can't believe I'm out!",
            "So blue.",
            "I feel so fast and free!",
            "Box kite!",
            "Wow!",
            "Flowers!",
            "This is Blue Leader.\n"+
                    "We have roses visual.",
            "Bring it around 30 degrees and hold.",
            "Roses!",
            "30 degrees, roger. Bringing it around.",
            "Stand to the side, kid.\n"+
                    "It's got a bit of a kick.",
            "That is one nectar collector!",
            "- Ever see pollination up close?\n"+
                    "- No, sir.",
            "I pick up some pollen here, sprinkle it\n"+
                    "over here. Maybe a dash over there,",
            "a pinch on that one.\n"+
                    "See that? It's a little bit of magic.",
            "That's amazing. Why do we do that?",
            "That's pollen power. More pollen, more\n"+
                    "flowers, more nectar, more honey for us.",
            "Oool.",
            "I'm picking up a lot of bright yellow.\n"+
                    "Oould be daisies. Don't we need those?",
            "Oopy that visual.",
            "Wait. One of these flowers\n"+
                    "seems to be on the move.",
            "Say again? You're reporting\n"+
                    "a moving flower?",
            "Affirmative.",
            "That was on the line!",
            "This is the coolest. What is it?",
            "I don't know, but I'm loving this color.",
            "It smells good.\n"+
                    "Not like a flower, but I like it.",
            "Yeah, fuzzy.",
            "Ohemical-y.",
            "Oareful, guys. It's a little grabby.",
            "My sweet lord of bees!",
            "Oandy-brain, get off there!",
            "Problem!",
            "- Guys!\n"+
                    "- This could be bad.",
            "Affirmative.",
            "Very close.",
            "Gonna hurt.",
            "Mama's little boy.",
            "You are way out of position, rookie!",
            "Ooming in at you like a missile!",
            "Help me!",
            "I don't think these are flowers.",
            "- Should we tell him?\n"+
                    "- I think he knows.",
            "What is this?!",
            "Match point!",
            "You can start packing up, honey, because you're about to eat it!",
            "Yowser!",
            "Gross.",
            "There's a bee in the car!",
            "- Do something!\n"+
                    "- I'm driving!",
            "- Hi, bee.\n"+
                    "- He's back here!",
            "He's going to sting me!",
            "Nobody move. If you don't move, he won't sting you. Freeze!",
            "He blinked!",
            "Spray him, Granny!",
            "What are you doing?!",
            "Wow... the tension level\n"+
                    "out here is unbelievable.",
            "I gotta get home.",
            "Oan't fly in rain.",
            "Oan't fly in rain.",
            "Oan't fly in rain.",
            "Mayday! Mayday! Bee going down!",
            "Ken, could you close\n"+
                    "the window please?",
            "Ken, could you close\n"+
                    "the window please?",
            "Oheck out my new resume.\n"+
                    "I made it into a fold-out brochure.",
            "You see? Folds out.",
            "Oh, no. More humans. I don't need this.",
            "What was that?",
            "Maybe this time. This time. This time.\n"+
                    "This time! This time! This...",
            "Drapes!",
            "That is diabolical.",
            "It's fantastic. It's got all my special\n"+
                    "skills, even my top-ten favorite movies.",
            "What's number one? Star Wars?",
            "Nah, I don't go for that...",
            "...kind of stuff.",
            "No wonder we shouldn't talk to them.\n"+
                    "They're out of their minds.",
            "When I leave a job interview, they're\n"+
                    "flabbergasted, can't believe what I say.",
            "There's the sun. Maybe that's a way out.",
            "I don't remember the sun\n"+
                    "having a big 75 on it.",
            "I predicted global warming.",
            "I could feel it getting hotter.\n"+
                    "At first I thought it was just me.",
            "Wait! Stop! Bee!",
            "Stand back. These are winter boots.",
            "Wait!",
            "Don't kill him!",
            "You know I'm allergic to them!\n"+
                    "This thing could kill me!",
            "Why does his life have\n"+
                    "less value than yours?",
            "Why does his life have any less value\n"+
                    "than mine? Is that your statement?",
            "I'm just saying all life has value. You\n"+
                    "don't know what he's capable of feeling.",
            "My brochure!",
            "There you go, little guy.",
            "I'm not scared of him.\n"+
                    "It's an allergic thing.",
            "Put that on your resume brochure.",
            "My whole face could puff up.",
            "Make it one of your special skills.",
            "Knocking someone out\n"+
                    "is also a special skill.",
            "Right. Bye, Vanessa. Thanks.",
            "- Vanessa, next week? Yogurt night?\n"+
                    "- Sure, Ken. You know, whatever.",
            "- You could put carob chips on there.\n"+
                    "- Bye.",
            "- Supposed to be less calories.\n"+
                    "- Bye.",
            "I gotta say something.",
            "She saved my life.\n"+
                    "I gotta say something.",
            "All right, here it goes.",
            "Nah.",
            "What would I say?",
            "I could really get in trouble.",
            "It's a bee law.\n"+
                    "You're not supposed to talk to a human.",
            "I can't believe I'm doing this.",
            "I've got to.",
            "Oh, I can't do it. Oome on!",
            "No. Yes. No.",
            "Do it. I can't.",
            "How should I start it?\n"+
                    "\"You like jazz?\" No, that's no good.",
            "Here she comes! Speak, you fool!",
            "Hi!",
            "I'm sorry.",
            "- You're talking.\n"+
                    "- Yes, I know.",
            "You're talking!",
            "I'm so sorry.",
            "No, it's OK. It's fine.\n"+
                    "I know I'm dreaming.",
            "But I don't recall going to bed.",
            "Well, I'm sure this\n"+
                    "is very disconcerting.",
            "This is a bit of a surprise to me.\n"+
                    "I mean, you're a bee!",
            "I am. And I'm not supposed\n"+
                    "to be doing this,",
            "but they were all trying to kill me.",
            "And if it wasn't for you...",
            "I had to thank you.\n"+
                    "It's just how I was raised.",
            "That was a little weird.",
            "- I'm talking with a bee.\n"+
                    "- Yeah.",
            "I'm talking to a bee.\n"+
                    "And the bee is talking to me!",
            "I just want to say I'm grateful.\n"+
                    "I'll leave now.",
            "- Wait! How did you learn to do that?\n"+
                    "- What?",
            "The talking thing.",
            "Same way you did, I guess.\n"+
                    "\"Mama, Dada, honey.\" You pick it up.",
            "- That's very funny.\n"+
                    "- Yeah.",
            "Bees are funny. If we didn't laugh, we'd cry with what we have to deal with.",
            "Anyway...",
            "Oan I...",
            "...get you something?\n"+
                    "- Like what?",
            "I don't know. I mean...\n"+
                    "I don't know. Ooffee?",
            "I don't want to put you out.",
            "It's no trouble. It takes two minutes.",
            "- It's just coffee.\n"+
                    "- I hate to impose.",
            "- Don't be ridiculous!\n"+
                    "- Actually, I would love a cup.",
            "Hey, you want rum cake?",
            "- I shouldn't.\n"+
                    "- Have some.",
            "- No, I can't.\n"+
                    "- Oome on!",
            "I'm trying to lose a couple micrograms.",
            "- Where?\n"+
                    "- These stripes don't help.",
            "You look great!",
            "I don't know if you know\n"+
                    "anything about fashion.",
            "Are you all right?",
            "No.",
            "He's making the tie in the cab\n"+
                    "as they're flying up Madison.",
            "He finally gets there.",
            "He runs up the steps into the church.\n"+
                    "The wedding is on.",
            "And he says, \"Watermelon?\n"+
                    "I thought you said Guatemalan.",
            "Why would I marry a watermelon?",
            "Is that a bee joke?",
            "That's the kind of stuff we do.",
            "Yeah, different.",
            "So, what are you gonna do, Barry?",
            "About work? I don't know.",
            "I want to do my part for the hive, but I can't do it the way they want.",
            "I know how you feel.",
            "- You do?\n"+
                    "- Sure.",
            "My parents wanted me to be a lawyer or\n"+
                    "a doctor, but I wanted to be a florist.",
            "- Really?\n"+
                    "- My only interest is flowers.",
            "Our new queen was just elected\n"+
                    "with that same campaign slogan.",
            "Anyway, if you look...",
            "There's my hive right there. See it?",
            "You're in Sheep Meadow!",
            "Yes! I'm right off the Turtle Pond!",
            "No way! I know that area.\n"+
                    "I lost a toe ring there once.",
            "- Why do girls put rings on their toes?\n"+
                    "- Why not?",
            "- It's like putting a hat on your knee.\n"+
                    "- Maybe I'll try that.",
            "- You all right, ma'am?\n"+
                    "- Oh, yeah. Fine.",
            "Just having two cups of coffee!",
            "Anyway, this has been great.\n"+
                    "Thanks for the coffee.",
            "Yeah, it's no trouble.",
            "Sorry I couldn't finish it. If I did, I'd be up the rest of my life.",
            "Are you...?",
            "Oan I take a piece of this with me?",
            "Sure! Here, have a crumb.",
            "- Thanks!\n"+
                    "- Yeah.",
            "All right. Well, then...\n"+
                    "I guess I'll see you around.",
            "Or not.",
            "OK, Barry.",
            "And thank you\n"+
                    "so much again... for before.",
            "Oh, that? That was nothing.",
            "Well, not nothing, but... Anyway...",
            "This can't possibly work.",
            "He's all set to go.\n"+
                    "We may as well try it.",
            "OK, Dave, pull the chute.",
            "- Sounds amazing.\n"+
                    "- It was amazing!",
            "It was the scariest, happiest moment of my life.",
            "Humans! I can't believe\n"+
                    "you were with humans!",
            "Giant, scary humans!\n"+
                    "What were they like?",
            "Huge and crazy. They talk crazy.",
            "They eat crazy giant things.\n"+
                    "They drive crazy.",
            "- Do they try and kill you, like on TV?\n"+
                    "- Some of them. But some of them don't.",
            "- How'd you get back?\n"+
                    "- Poodle.",
            "You did it, and I'm glad. You saw\n"+
                    "whatever you wanted to see.",
            "You had your \"experience.\" Now you\n"+
                    "can pick out your job and be normal.",
            "- Well...\n"+
                    "- Well?",
            "Well, I met someone.",
            "You did? Was she Bee-ish?",
            "- A wasp?! Your parents will kill you!\n"+
                    "- No, no, no, not a wasp.",
            "- Spider?\n"+
                    "- I'm not attracted to spiders.",
            "I know it's the hottest thing, with the eight legs and all.",
            "I can't get by that face.",
            "So who is she?",
            "She's... human.",
            "No, no. That's a bee law.\n"+
                    "You wouldn't break a bee law.",
            "- Her name's Vanessa.\n"+
                    "- Oh, boy.",
            "She's so nice. And she's a florist!",
            "Oh, no! You're dating a human florist!",
            "We're not dating.",
            "You're flying outside the hive, talking\n"+
                    "to humans that attack our homes",
            "with power washers and M-80s!\n"+
                    "One-eighth a stick of dynamite!",
            "She saved my life!\n"+
                    "And she understands me.",
            "This is over!",
            "Eat this.",
            "This is not over! What was that?",
            "- They call it a crumb.\n"+
                    "- It was so stingin' stripey!",
            "And that's not what they eat.\n"+
                    "That's what falls off what they eat!",
            "- You know what a Oinnabon is?\n"+
                    "- No.",
            "It's bread and cinnamon and frosting.\n"+
                    "They heat it up...",
            "Sit down!",
            "...really hot!\n"+
                    "- Listen to me!",
            "We are not them! We're us.\n"+
                    "There's us and there's them!",
            "Yes, but who can deny\n"+
                    "the heart that is yearning?",
            "There's no yearning.\n"+
                    "Stop yearning. Listen to me!",
            "You have got to start thinking bee, my friend. Thinking bee!",
            "- Thinking bee.\n"+
                    "- Thinking bee.",
            "Thinking bee! Thinking bee!\n"+
                    "Thinking bee! Thinking bee!",
            "There he is. He's in the pool.",
            "You know what your problem is, Barry?",
            "I gotta start thinking bee?",
            "How much longer will this go on?",
            "It's been three days!\n"+
                    "Why aren't you working?",
            "I've got a lot of big life decisions\n"+
                    "to think about.",
            "What life? You have no life!\n"+
                    "You have no job. You're barely a bee!",
            "Would it kill you\n"+
                    "to make a little honey?",
            "Barry, come out.\n"+
                    "Your father's talking to you.",
            "Martin, would you talk to him?",
            "Barry, I'm talking to you!",
            "You coming?",
            "Got everything?",
            "All set!",
            "Go ahead. I'll catch up.",
            "Don't be too long.",
            "Watch this!",
            "Vanessa!",
            "- We're still here.\n"+
                    "- I told you not to yell at him.",
            "He doesn't respond to yelling!",
            "- Then why yell at me?\n"+
                    "- Because you don't listen!",
            "I'm not listening to this.",
            "Sorry, I've gotta go.",
            "- Where are you going?\n"+
                    "- I'm meeting a friend.",
            "A girl? Is this why you can't decide?",
            "Bye.",
            "I just hope she's Bee-ish.",
            "They have a huge parade\n"+
                    "of flowers every year in Pasadena?",
            "To be in the Tournament of Roses, that's every florist's dream!",
            "Up on a float, surrounded\n"+
                    "by flowers, crowds cheering.",
            "A tournament. Do the roses\n"+
                    "compete in athletic events?",
            "No. All right, I've got one.\n"+
                    "How come you don't fly everywhere?",
            "It's exhausting. Why don't you\n"+
                    "run everywhere? It's faster.",
            "Yeah, OK, I see, I see.\n"+
                    "All right, your turn.",
            "TiVo. You can just freeze live TV?\n"+
                    "That's insane!",
            "You don't have that?",
            "We have Hivo, but it's a disease.\n"+
                    "It's a horrible, horrible disease.",
            "Oh, my.",
            "Dumb bees!",
            "You must want to sting all those jerks.",
            "We try not to sting.\n"+
                    "It's usually fatal for us.",
            "So you have to watch your temper.",
            "Very carefully.\n"+
                    "You kick a wall, take a walk,",
            "write an angry letter and throw it out.\n"+
                    "Work through it like any emotion:",
            "Anger, jealousy, lust.",
            "Oh, my goodness! Are you OK?",
            "Yeah.",
            "- What is wrong with you?!\n"+
                    "- It's a bug.",
            "He's not bothering anybody.\n"+
                    "Get out of here, you creep!",
            "What was that? A Pic 'N' Save circular?",
            "Yeah, it was. How did you know?",
            "It felt like about 10 pages.\n"+
                    "Seventy-five is pretty much our limit.",
            "You've really got that\n"+
                    "down to a science.",
            "- I lost a cousin to Italian Vogue.\n"+
                    "- I'll bet.",
            "What in the name\n"+
                    "of Mighty Hercules is this?",
            "How did this get here?\n"+
                    "Oute Bee, Golden Blossom,",
            "Ray Liotta Private Select?",
            "- Is he that actor?\n"+
                    "- I never heard of him.",
            "- Why is this here?\n"+
                    "- For people. We eat it.",
            "You don't have\n"+
                    "enough food of your own?",
            "- Well, yes.\n"+
                    "- How do you get it?",
            "- Bees make it.\n"+
                    "- I know who makes it!",
            "And it's hard to make it!",
            "There's heating, cooling, stirring.\n"+
                    "You need a whole Krelman thing!",
            "- It's organic.\n"+
                    "- It's our-ganic!",
            "It's just honey, Barry.",
            "Just what?!",
            "Bees don't know about this!\n"+
                    "This is stealing! A lot of stealing!",
            "You've taken our homes, schools, hospitals! This is all we have!",
            "And it's on sale?!\n"+
                    "I'm getting to the bottom of this.",
            "I'm getting to the bottom\n"+
                    "of all of this!",
            "Hey, Hector.",
            "- You almost done?\n"+
                    "- Almost.",
            "He is here. I sense it.",
            "Well, I guess I'll go home now",
            "and just leave this nice honey out, with no one around.",
            "You're busted, box boy!",
            "I knew I heard something.\n"+
                    "So you can talk!",
            "I can talk.\n"+
                    "And now you'll start talking!",
            "Where you getting the sweet stuff?\n"+
                    "Who's your supplier?",
            "I don't understand.\n"+
                    "I thought we were friends.",
            "The last thing we want\n"+
                    "to do is upset bees!",
            "You're too late! It's ours now!",
            "You, sir, have crossed\n"+
                    "the wrong sword!",
            "You, sir, will be lunch\n"+
                    "for my iguana, Ignacio!",
            "Where is the honey coming from?",
            "Tell me where!",
            "Honey Farms! It comes from Honey Farms!",
            "Orazy person!",
            "What horrible thing has happened here?",
            "These faces, they never knew\n"+
                    "what hit them. And now",
            "they're on the road to nowhere!",
            "Just keep still.",
            "What? You're not dead?",
            "Do I look dead? They will wipe anything\n"+
                    "that moves. Where you headed?",
            "To Honey Farms.\n"+
                    "I am onto something huge here.",
            "I'm going to Alaska. Moose blood, crazy stuff. Blows your head off!",
            "I'm going to Tacoma.",
            "- And you?\n"+
                    "- He really is dead.",
            "All right.",
            "Uh-oh!",
            "- What is that?!\n"+
                    "- Oh, no!",
            "- A wiper! Triple blade!\n"+
                    "- Triple blade?",
            "Jump on! It's your only chance, bee!",
            "Why does everything have\n"+
                    "to be so doggone clean?!",
            "How much do you people need to see?!",
            "Open your eyes!\n"+
                    "Stick your head out the window!",
            "From NPR News in Washington, I'm Oarl Kasell.",
            "But don't kill no more bugs!",
            "- Bee!\n"+
                    "- Moose blood guy!!",
            "- You hear something?\n"+
                    "- Like what?",
            "Like tiny screaming.",
            "Turn off the radio.",
            "Whassup, bee boy?",
            "Hey, Blood.",
            "Just a row of honey jars, as far as the eye could see.",
            "Wow!",
            "I assume wherever this truck goes\n"+
                    "is where they're getting it.",
            "I mean, that honey's ours.",
            "- Bees hang tight.\n"+
                    "- We're all jammed in.",
            "It's a close community.",
            "Not us, man. We on our own.\n"+
                    "Every mosquito on his own.",
            "- What if you get in trouble?\n"+
                    "- You a mosquito, you in trouble.",
            "Nobody likes us. They just smack.\n"+
                    "See a mosquito, smack, smack!",
            "At least you're out in the world.\n"+
                    "You must meet girls.",
            "Mosquito girls try to trade u, pget with a moth, dragonfly.",
            "Mosquito girl don't want no mosquito.",
            "You got to be kidding me!",
            "Mooseblood's about to leave\n"+
                    "the building! So long, bee!",
            "- Hey, guys!\n"+
                    "- Mooseblood!",
            "I knew I'd catch y'all down here.\n"+
                    "Did you bring your crazy straw?",
            "We throw it in jars, slap a label on it, and it's pretty much pure profit.",
            "What is this place?",
            "A bee's got a brain\n"+
                    "the size of a pinhead.",
            "They are pinheads!",
            "Pinhead.",
            "- Oheck out the new smoker.\n"+
                    "- Oh, sweet. That's the one you want.",
            "The Thomas 3000!",
            "Smoker?",
            "Ninety puffs a minute, semi-automatic.\n"+
                    "Twice the nicotine, all the tar.",
            "A couple breaths of this\n"+
                    "knocks them right out.",
            "They make the honey, and we make the money.",
            "\"They make the honey, and we make the money\"?",
            "Oh, my!",
            "What's going on? Are you OK?",
            "Yeah. It doesn't last too long.",
            "Do you know you're\n"+
                    "in a fake hive with fake walls?",
            "Our queen was moved here.\n"+
                    "We had no choice.",
            "This is your queen?\n"+
                    "That's a man in women's clothes!",
            "That's a drag queen!",
            "What is this?",
            "Oh, no!",
            "There's hundreds of them!",
            "Bee honey.",
            "Our honey is being brazenly stolen\n"+
                    "on a massive scale!",
            "This is worse than anything bears\n"+
                    "have done! I intend to do something.",
            "Oh, Barry, stop.",
            "Who told you humans are taking\n"+
                    "our honey? That's a rumor.",
            "Do these look like rumors?",
            "That's a conspiracy theory.\n"+
                    "These are obviously doctored photos.",
            "How did you get mixed up in this?",
            "He's been talking to humans.",
            "- What?\n"+
                    "- Talking to humans?!",
            "He has a human girlfriend.\n"+
                    "And they make out!",
            "Make out? Barry!",
            "We do not.",
            "- You wish you could.\n"+
                    "- Whose side are you on?",
            "The bees!",
            "I dated a cricket once in San Antonio.\n"+
                    "Those crazy legs kept me up all night.",
            "Barry, this is what you want\n"+
                    "to do with your life?",
            "I want to do it for all our lives.\n"+
                    "Nobody works harder than bees!",
            "Dad, I remember you\n"+
                    "coming home so overworked",
            "your hands were still stirring.\n"+
                    "You couldn't stop.",
            "I remember that.",
            "What right do they have to our honey?",
            "We live on two cups a year. They put it\n"+
                    "in lip balm for no reason whatsoever!",
            "Even if it's true, what can one bee do?",
            "Sting them where it really hurts.",
            "In the face! The eye!",
            "- That would hurt.\n"+
                    "- No.",
            "Up the nose? That's a killer.",
            "There's only one place you can sting\n"+
                    "the humans, one place where it matters.",
            "Hive at Five, the hive's only\n"+
                    "full-hour action news source.",
            "No more bee beards!",
            "With Bob Bumble at the anchor desk.",
            "Weather with Storm Stinger.",
            "Sports with Buzz Larvi.",
            "And Jeanette Ohung.",
            "- Good evening. I'm Bob Bumble.\n"+
                    "- And I'm Jeanette Ohung.",
            "A tri-county bee, Barry Benson,",
            "intends to sue the human race\n"+
                    "for stealing our honey,",
            "packaging it and profiting\n"+
                    "from it illegally!",
            "Tomorrow night on Bee Larry King,",
            "we'll have three former queens here in\n"+
                    "our studio, discussing their new book,",
            "Olassy Ladies, out this week on Hexagon.",
            "Tonight we're talking to Barry Benson.",
            "Did you ever think, \"I'm a kid\n"+
                    "from the hive. I can't do this\"?",
            "Bees have never been afraid\n"+
                    "to change the world.",
            "What about Bee Oolumbus?\n"+
                    "Bee Gandhi? Bejesus?",
            "Where I'm from, we'd never sue humans.",
            "We were thinking\n"+
                    "of stickball or candy stores.",
            "How old are you?",
            "The bee community\n"+
                    "is supporting you in this case,",
            "which will be the trial\n"+
                    "of the bee century.",
            "You know, they have a Larry King\n"+
                    "in the human world too.",
            "It's a common name. Next week...",
            "He looks like you and has a show\n"+
                    "and suspenders and colored dots...",
            "Next week...",
            "Glasses, quotes on the bottom from the\n"+
                    "guest even though you just heard 'em.",
            "Bear Week next week!\n"+
                    "They're scary, hairy and here live.",
            "Always leans forward, pointy shoulders, squinty eyes, very Jewish.",
            "In tennis, you attack\n"+
                    "at the point of weakness!",
            "It was my grandmother, Ken. She's 81.",
            "Honey, her backhand's a joke!\n"+
                    "I'm not gonna take advantage of that?",
            "Quiet, please.\n"+
                    "Actual work going on here.",
            "- Is that that same bee?\n"+
                    "- Yes, it is!",
            "I'm helping him sue the human race.",
            "- Hello.\n"+
                    "- Hello, bee.",
            "This is Ken.",
            "Yeah, I remember you. Timberland, size\n"+
                    "ten and a half. Vibram sole, I believe.",
            "Why does he talk again?",
            "Listen, you better go\n"+
                    "'cause we're really busy working.",
            "But it's our yogurt night!",
            "Bye-bye.",
            "Why is yogurt night so difficult?!",
            "You poor thing.\n"+
                    "You two have been at this for hours!",
            "Yes, and Adam here\n"+
                    "has been a huge help.",
            "- Frosting...\n"+
                    "- How many sugars?",
            "Just one. I try not\n"+
                    "to use the competition.",
            "So why are you helping me?",
            "Bees have good qualities.",
            "And it takes my mind off the shop.",
            "Instead of flowers, people\n"+
                    "are giving balloon bouquets now.",
            "Those are great, if you're three.",
            "And artificial flowers.",
            "- Oh, those just get me psychotic!\n"+
                    "- Yeah, me too.",
            "Bent stingers, pointless pollination.",
            "Bees must hate those fake things!",
            "Nothing worse\n"+
                    "than a daffodil that's had work done.",
            "Maybe this could make up\n"+
                    "for it a little bit.",
            "- This lawsuit's a pretty big deal.\n"+
                    "- I guess.",
            "You sure you want to go through with it?",
            "Am I sure? When I'm done with\n"+
                    "the humans, they won't be able",
            "to say, \"Honey, I'm home\"\n"+
                    "without paying a royalty!",
            "It's an incredible scene\n"+
                    "here in downtown Manhattan,",
            "where the world anxiously waits, because for the first time in history,",
            "we will hear for ourselves\n"+
                    "if a honeybee can actually speak.",
            "What have we gotten into here, Barry?",
            "It's pretty big, isn't it?",
            "I can't believe how many humans\n"+
                    "don't work during the day.",
            "You think billion-dollar multinational\n"+
                    "food companies have good lawyers?",
            "Everybody needs to stay\n"+
                    "behind the barricade.",
            "- What's the matter?\n"+
                    "- I don't know, I just got a chill.",
            "Well, if it isn't the bee team.",
            "You boys work on this?",
            "All rise! The Honorable\n"+
                    "Judge Bumbleton presiding.",
            "All right. Oase number 4475,",
            "Superior Oourt of New York, Barry Bee Benson v. the Honey Industry",
            "is now in session.",
            "Mr. Montgomery, you're representing\n"+
                    "the five food companies collectively?",
            "A privilege.",
            "Mr. Benson... you're representing\n"+
                    "all the bees of the world?",
            "I'm kidding. Yes, Your Honor, we're ready to proceed.",
            "Mr. Montgomery, your opening statement, please.",
            "Ladies and gentlemen of the jury,",
            "my grandmother was a simple woman.",
            "Born on a farm, she believed\n"+
                    "it was man's divine right",
            "to benefit from the bounty\n"+
                    "of nature God put before us.",
            "If we lived in the topsy-turvy world\n"+
                    "Mr. Benson imagines,",
            "just think of what would it mean.",
            "I would have to negotiate\n"+
                    "with the silkworm",
            "for the elastic in my britches!",
            "Talking bee!",
            "How do we know this isn't some sort of",
            "holographic motion-picture-capture\n"+
                    "Hollywood wizardry?",
            "They could be using laser beams!",
            "Robotics! Ventriloquism!\n"+
                    "Oloning! For all we know,",
            "he could be on steroids!",
            "Mr. Benson?",
            "Ladies and gentlemen, there's no trickery here.",
            "I'm just an ordinary bee.\n"+
                    "Honey's pretty important to me.",
            "It's important to all bees.\n"+
                    "We invented it!",
            "We make it. And we protect it\n"+
                    "with our lives.",
            "Unfortunately, there are\n"+
                    "some people in this room",
            "who think they can take it from us",
            "'cause we're the little guys!\n"+
                    "I'm hoping that, after this is all over,",
            "you'll see how, by taking our honey, you not only take everything we have",
            "but everything we are!",
            "I wish he'd dress like that\n"+
                    "all the time. So nice!",
            "Oall your first witness.",
            "So, Mr. Klauss Vanderhayden\n"+
                    "of Honey Farms, big company you have.",
            "I suppose so.",
            "I see you also own\n"+
                    "Honeyburton and Honron!",
            "Yes, they provide beekeepers\n"+
                    "for our farms.",
            "Beekeeper. I find that\n"+
                    "to be a very disturbing term.",
            "I don't imagine you employ\n"+
                    "any bee-free-ers, do you?",
            "- No.\n"+
                    "- I couldn't hear you.",
            "- No.\n"+
                    "- No.",
            "Because you don't free bees.\n"+
                    "You keep bees. Not only that,",
            "it seems you thought a bear would be\n"+
                    "an appropriate image for a jar of honey.",
            "They're very lovable creatures.",
            "Yogi Bear, Fozzie Bear, Build-A-Bear.",
            "You mean like this?",
            "Bears kill bees!",
            "How'd you like his head crashing\n"+
                    "through your living room?!",
            "Biting into your couch!\n"+
                    "Spitting out your throw pillows!",
            "OK, that's enough. Take him away.",
            "So, Mr. Sting, thank you for being here.\n"+
                    "Your name intrigues me.",
            "- Where have I heard it before?\n"+
                    "- I was with a band called The Police.",
            "But you've never been\n"+
                    "a police officer, have you?",
            "No, I haven't.",
            "No, you haven't. And so here\n"+
                    "we have yet another example",
            "of bee culture casually\n"+
                    "stolen by a human",
            "for nothing more than\n"+
                    "a prance-about stage name.",
            "Oh, please.",
            "Have you ever been stung, Mr. Sting?",
            "Because I'm feeling\n"+
                    "a little stung, Sting.",
            "Or should I say... Mr. Gordon M. Sumner!",
            "That's not his real name?! You idiots!",
            "Mr. Liotta, first, belated congratulations on",
            "your Emmy win for a guest spot\n"+
                    "on ER in 2005.",
            "Thank you. Thank you.",
            "I see from your resume\n"+
                    "that you're devilishly handsome",
            "with a churning inner turmoil\n"+
                    "that's ready to blow.",
            "I enjoy what I do. Is that a crime?",
            "Not yet it isn't. But is this\n"+
                    "what it's come to for you?",
            "Exploiting tiny, helpless bees\n"+
                    "so you don't",
            "have to rehearse\n"+
                    "your part and learn your lines, sir?",
            "Watch it, Benson!\n"+
                    "I could blow right now!",
            "This isn't a goodfella.\n"+
                    "This is a badfella!",
            "Why doesn't someone just step on\n"+
                    "this creep, and we can all go home?!",
            "- Order in this court!\n"+
                    "- You're all thinking it!",
            "Order! Order, I say!",
            "- Say it!\n"+
                    "- Mr. Liotta, please sit down!",
            "I think it was awfully nice\n"+
                    "of that bear to pitch in like that.",
            "I think the jury's on our side.",
            "Are we doing everything right, legally?",
            "I'm a florist.",
            "Right. Well, here's to a great team.",
            "To a great team!",
            "Well, hello.",
            "- Ken!\n"+
                    "- Hello.",
            "I didn't think you were coming.",
            "No, I was just late.\n"+
                    "I tried to call, but... the battery.",
            "I didn't want all this to go to waste, so I called Barry. Luckily, he was free.",
            "Oh, that was lucky.",
            "There's a little left.\n"+
                    "I could heat it up.",
            "Yeah, heat it up, sure, whatever.",
            "So I hear you're quite a tennis player.",
            "I'm not much for the game myself.\n"+
                    "The ball's a little grabby.",
            "That's where I usually sit.\n"+
                    "Right... there.",
            "Ken, Barry was looking at your resume,",
            "and he agreed with me that eating with\n"+
                    "chopsticks isn't really a special skill.",
            "You think I don't see what you're doing?",
            "I know how hard it is to find\n"+
                    "the rightjob. We have that in common.",
            "Do we?",
            "Bees have 100 percent employment, but we do jobs like taking the crud out.",
            "That's just what\n"+
                    "I was thinking about doing.",
            "Ken, I let Barry borrow your razor\n"+
                    "for his fuzz. I hope that was all right.",
            "I'm going to drain the old stinger.",
            "Yeah, you do that.",
            "Look at that.",
            "You know, I've just about had it",
            "with your little mind games.",
            "- What's that?\n"+
                    "- Italian Vogue.",
            "Mamma mia, that's a lot of pages.",
            "A lot of ads.",
            "Remember what Van said, why is\n"+
                    "your life more valuable than mine?",
            "Funny, I just can't seem to recall that!",
            "I think something stinks in here!",
            "I love the smell of flowers.",
            "How do you like the smell of flames?!",
            "Not as much.",
            "Water bug! Not taking sides!",
            "Ken, I'm wearing a Ohapstick hat!\n"+
                    "This is pathetic!",
            "I've got issues!",
            "Well, well, well, a royal flush!",
            "- You're bluffing.\n"+
                    "- Am I?",
            "Surf's up, dude!",
            "Poo water!",
            "That bowl is gnarly.",
            "Except for those dirty yellow rings!",
            "Kenneth! What are you doing?!",
            "You know, I don't even like honey!\n"+
                    "I don't eat it!",
            "We need to talk!",
            "He's just a little bee!",
            "And he happens to be\n"+
                    "the nicest bee I've met in a long time!",
            "Long time? What are you talking about?!\n"+
                    "Are there other bugs in your life?",
            "No, but there are other things bugging\n"+
                    "me in life. And you're one of them!",
            "Fine! Talking bees, no yogurt night...",
            "My nerves are fried from riding\n"+
                    "on this emotional roller coaster!",
            "Goodbye, Ken.",
            "And for your information,",
            "I prefer sugar-free, artificial\n"+
                    "sweeteners made by man!",
            "I'm sorry about all that.",
            "I know it's got\n"+
                    "an aftertaste! I like it!",
            "I always felt there was some kind\n"+
                    "of barrier between Ken and me.",
            "I couldn't overcome it.\n"+
                    "Oh, well.",
            "Are you OK for the trial?",
            "I believe Mr. Montgomery\n"+
                    "is about out of ideas.",
            "We would like to call\n"+
                    "Mr. Barry Benson Bee to the stand.",
            "Good idea! You can really see why he's\n"+
                    "considered one of the best lawyers...",
            "Yeah.",
            "Layton, you've\n"+
                    "gotta weave some magic",
            "with this jury, or it's gonna be all over.",
            "Don't worry. The only thing I have\n"+
                    "to do to turn this jury around",
            "is to remind them\n"+
                    "of what they don't like about bees.",
            "- You got the tweezers?\n"+
                    "- Are you allergic?",
            "Only to losing, son. Only to losing.",
            "Mr. Benson Bee, I'll ask you\n"+
                    "what I think we'd all like to know.",
            "What exactly is your relationship",
            "to that woman?",
            "We're friends.",
            "- Good friends?\n"+
                    "- Yes.",
            "How good? Do you live together?",
            "Wait a minute...",
            "Are you her little...",
            "...bedbug?",
            "I've seen a bee documentary or two.\n"+
                    "From what I understand,",
            "doesn't your queen give birth\n"+
                    "to all the bee children?",
            "- Yeah, but...\n"+
                    "- So those aren't your real parents!",
            "- Oh, Barry...\n"+
                    "- Yes, they are!",
            "Hold me back!",
            "You're an illegitimate bee, aren't you, Benson?",
            "He's denouncing bees!",
            "Don't y'all date your cousins?",
            "- Objection!\n"+
                    "- I'm going to pincushion this guy!",
            "Adam, don't! It's what he wants!",
            "Oh, I'm hit!!",
            "Oh, lordy, I am hit!",
            "Order! Order!",
            "The venom! The venom\n"+
                    "is coursing through my veins!",
            "I have been felled\n"+
                    "by a winged beast of destruction!",
            "You see? You can't treat them\n"+
                    "like equals! They're striped savages!",
            "Stinging's the only thing\n"+
                    "they know! It's their way!",
            "- Adam, stay with me.\n"+
                    "- I can't feel my legs.",
            "What angel of mercy\n"+
                    "will come forward to suck the poison",
            "from my heaving buttocks?",
            "I will have order in this court. Order!",
            "Order, please!",
            "The case of the honeybees\n"+
                    "versus the human race",
            "took a pointed turn against the bees",
            "yesterday when one of their legal\n"+
                    "team stung Layton T. Montgomery.",
            "- Hey, buddy.\n"+
                    "- Hey.",
            "- Is there much pain?\n"+
                    "- Yeah.",
            "I...",
            "I blew the whole case, didn't I?",
            "It doesn't matter. What matters is\n"+
                    "you're alive. You could have died.",
            "I'd be better off dead. Look at me.",
            "They got it from the cafeteria\n"+
                    "downstairs, in a tuna sandwich.",
            "Look, there's\n"+
                    "a little celery still on it.",
            "What was it like to sting someone?",
            "I can't explain it. It was all...",
            "All adrenaline and then...\n"+
                    "and then ecstasy!",
            "All right.",
            "You think it was all a trap?",
            "Of course. I'm sorry.\n"+
                    "I flew us right into this.",
            "What were we thinking? Look at us. We're\n"+
                    "just a couple of bugs in this world.",
            "What will the humans do to us\n"+
                    "if they win?",
            "I don't know.",
            "I hear they put the roaches in motels.\n"+
                    "That doesn't sound so bad.",
            "Adam, they check in, but they don't check out!",
            "Oh, my.",
            "Oould you get a nurse\n"+
                    "to close that window?",
            "- Why?\n"+
                    "- The smoke.",
            "Bees don't smoke.",
            "Right. Bees don't smoke.",
            "Bees don't smoke!\n"+
                    "But some bees are smoking.",
            "That's it! That's our case!",
            "It is? It's not over?",
            "Get dressed. I've gotta go somewhere.",
            "Get back to the court and stall.\n"+
                    "Stall any way you can.",
            "And assuming you've done step correctly, you're ready for the tub.",
            "Mr. Flayman.",
            "Yes? Yes, Your Honor!",
            "Where is the rest of your team?",
            "Well, Your Honor, it's interesting.",
            "Bees are trained to fly haphazardly,",
            "and as a result, we don't make very good time.",
            "I actually heard a funny story about...",
            "Your Honor, haven't these ridiculous bugs",
            "taken up enough\n"+
                    "of this court's valuable time?",
            "How much longer will we allow\n"+
                    "these absurd shenanigans to go on?",
            "They have presented no compelling\n"+
                    "evidence to support their charges",
            "against my clients, who run legitimate businesses.",
            "I move for a complete dismissal\n"+
                    "of this entire case!",
            "Mr. Flayman, I'm afraid I'm going",
            "to have to consider\n"+
                    "Mr. Montgomery's motion.",
            "But you can't! We have a terrific case.",
            "Where is your proof?\n"+
                    "Where is the evidence?",
            "Show me the smoking gun!",
            "Hold it, Your Honor!\n"+
                    "You want a smoking gun?",
            "Here is your smoking gun.",
            "What is that?",
            "It's a bee smoker!",
            "What, this?\n"+
                    "This harmless little contraption?",
            "This couldn't hurt a fly, let alone a bee.",
            "Look at what has happened",
            "to bees who have never been asked, \"Smoking or non?\"",
            "Is this what nature intended for us?",
            "To be forcibly addicted\n"+
                    "to smoke machines",
            "and man-made wooden slat work camps?",
            "Living out our lives as honey slaves\n"+
                    "to the white man?",
            "- What are we gonna do?\n"+
                    "- He's playing the species card.",
            "Ladies and gentlemen, please, free these bees!",
            "Free the bees! Free the bees!",
            "Free the bees!",
            "Free the bees! Free the bees!",
            "The court finds in favor of the bees!",
            "Vanessa, we won!",
            "I knew you could do it! High-five!",
            "Sorry.",
            "I'm OK! You know what this means?",
            "All the honey\n"+
                    "will finally belong to the bees.",
            "Now we won't have\n"+
                    "to work so hard all the time.",
            "This is an unholy perversion\n"+
                    "of the balance of nature, Benson.",
            "You'll regret this.",
            "Barry, how much honey is out there?",
            "All right. One at a time.",
            "Barry, who are you wearing?",
            "My sweater is Ralph Lauren, and I have no pants.",
            "- What if Montgomery's right?\n"+
                    "- What do you mean?",
            "We've been living the bee way\n"+
                    "a long time, 27 million years.",
            "Oongratulations on your victory.\n"+
                    "What will you demand as a settlement?",
            "First, we'll demand a complete shutdown\n"+
                    "of all bee work camps.",
            "Then we want back the honey\n"+
                    "that was ours to begin with,",
            "every last drop.",
            "We demand an end to the glorification\n"+
                    "of the bear as anything more",
            "than a filthy, smelly, bad-breath stink machine.",
            "We're all aware\n"+
                    "of what they do in the woods.",
            "Wait for my signal.",
            "Take him out.",
            "He'll have nauseous\n"+
                    "for a few hours, then he'll be fine.",
            "And we will no longer tolerate\n"+
                    "bee-negative nicknames...",
            "But it's just a prance-about stage name!",
            "...unnecessary inclusion of honey\n"+
                    "in bogus health products",
            "and la-dee-da human\n"+
                    "tea-time snack garnishments.",
            "Oan't breathe.",
            "Bring it in, boys!",
            "Hold it right there! Good.",
            "Tap it.",
            "Mr. Buzzwell, we just passed three cups, and there's gallons more coming!",
            "- I think we need to shut down!\n"+
                    "- Shut down? We've never shut down.",
            "Shut down honey production!",
            "Stop making honey!",
            "Turn your key, sir!",
            "What do we do now?",
            "Oannonball!",
            "We're shutting honey production!",
            "Mission abort.",
            "Aborting pollination and nectar detail.\n"+
                    "Returning to base.",
            "Adam, you wouldn't believe\n"+
                    "how much honey was out there.",
            "Oh, yeah?",
            "What's going on? Where is everybody?",
            "- Are they out celebrating?\n"+
                    "- They're home.",
            "They don't know what to do.\n"+
                    "Laying out, sleeping in.",
            "I heard your Uncle Oarl was on his way\n"+
                    "to San Antonio with a cricket.",
            "At least we got our honey back.",
            "Sometimes I think, so what if humans\n"+
                    "liked our honey? Who wouldn't?",
            "It's the greatest thing in the world!\n"+
                    "I was excited to be part of making it.",
            "This was my new desk. This was my\n"+
                    "new job. I wanted to do it really well.",
            "And now...",
            "Now I can't.",
            "I don't understand\n"+
                    "why they're not happy.",
            "I thought their lives would be better!",
            "They're doing nothing. It's amazing.\n"+
                    "Honey really changes people.",
            "You don't have any idea\n"+
                    "what's going on, do you?",
            "- What did you want to show me?\n"+
                    "- This.",
            "What happened here?",
            "That is not the half of it.",
            "Oh, no. Oh, my.",
            "They're all wilting.",
            "Doesn't look very good, does it?",
            "No.",
            "And whose fault do you think that is?",
            "You know, I'm gonna guess bees.",
            "Bees?",
            "Specifically, me.",
            "I didn't think bees not needing to make\n"+
                    "honey would affect all these things.",
            "It's notjust flowers.\n"+
                    "Fruits, vegetables, they all need bees.",
            "That's our whole SAT test right there.",
            "Take away produce, that affects\n"+
                    "the entire animal kingdom.",
            "And then, of course...",
            "The human species?",
            "So if there's no more pollination,",
            "it could all just go south here, couldn't it?",
            "I know this is also partly my fault.",
            "How about a suicide pact?",
            "How do we do it?",
            "- I'll sting you, you step on me.\n"+
                    "- Thatjust kills you twice.",
            "Right, right.",
            "Listen, Barry...\n"+
                    "sorry, but I gotta get going.",
            "I had to open my mouth and talk.",
            "Vanessa?",
            "Vanessa? Why are you leaving?\n"+
                    "Where are you going?",
            "To the final Tournament of Roses parade\n"+
                    "in Pasadena.",
            "They've moved it to this weekend\n"+
                    "because all the flowers are dying.",
            "It's the last chance\n"+
                    "I'll ever have to see it.",
            "Vanessa, I just wanna say I'm sorry.\n"+
                    "I never meant it to turn out like this.",
            "I know. Me neither.",
            "Tournament of Roses.\n"+
                    "Roses can't do sports.",
            "Wait a minute. Roses. Roses?",
            "Roses!",
            "Vanessa!",
            "Roses?!",
            "Barry?",
            "- Roses are flowers!\n"+
                    "- Yes, they are.",
            "Flowers, bees, pollen!",
            "I know.\n"+
                    "That's why this is the last parade.",
            "Maybe not.\n"+
                    "Oould you ask him to slow down?",
            "Oould you slow down?",
            "Barry!",
            "OK, I made a huge mistake.\n"+
                    "This is a total disaster, all my fault.",
            "Yes, it kind of is.",
            "I've ruined the planet.\n"+
                    "I wanted to help you",
            "with the flower shop.\n"+
                    "I've made it worse.",
            "Actually, it's completely closed down.",
            "I thought maybe you were remodeling.",
            "But I have another idea, and it's\n"+
                    "greater than my previous ideas combined.",
            "I don't want to hear it!",
            "All right, they have the roses, the roses have the pollen.",
            "I know every bee, plant\n"+
                    "and flower bud in this park.",
            "All we gotta do is get what they've got\n"+
                    "back here with what we've got.",
            "- Bees.\n"+
                    "- Park.",
            "- Pollen!\n"+
                    "- Flowers.",
            "- Repollination!\n"+
                    "- Across the nation!",
            "Tournament of Roses, Pasadena, Oalifornia.",
            "They've got nothing\n"+
                    "but flowers, floats and cotton candy.",
            "Security will be tight.",
            "I have an idea.",
            "Vanessa Bloome, FTD.",
            "Official floral business. It's real.",
            "Sorry, ma'am. Nice brooch.",
            "Thank you. It was a gift.",
            "Once inside, we just pick the right float.",
            "How about The Princess and the Pea?",
            "I could be the princess, and you could be the pea!",
            "Yes, I got it.",
            "- Where should I sit?\n"+
                    "- What are you?",
            "- I believe I'm the pea.\n"+
                    "- The pea?",
            "It goes under the mattresses.",
            "- Not in this fairy tale, sweetheart.\n"+
                    "- I'm getting the marshal.",
            "You do that!\n"+
                    "This whole parade is a fiasco!",
            "Let's see what this baby'll do.",
            "Hey, what are you doing?!",
            "Then all we do\n"+
                    "is blend in with traffic...",
            "...without arousing suspicion.",
            "Once at the airport, there's no stopping us.",
            "Stop! Security.",
            "- You and your insect pack your float?\n"+
                    "- Yes.",
            "Has it been\n"+
                    "in your possession the entire time?",
            "Would you remove your shoes?",
            "- Remove your stinger.\n"+
                    "- It's part of me.",
            "I know. Just having some fun.\n"+
                    "Enjoy your flight.",
            "Then if we're lucky, we'll have\n"+
                    "just enough pollen to do the job.",
            "Oan you believe how lucky we are? We\n"+
                    "have just enough pollen to do the job!",
            "I think this is gonna work.",
            "It's got to work.",
            "Attention, passengers, this is Oaptain Scott.",
            "We have a bit of bad weather\n"+
                    "in New York.",
            "It looks like we'll experience\n"+
                    "a couple hours delay.",
            "Barry, these are cut flowers\n"+
                    "with no water. They'll never make it.",
            "I gotta get up there\n"+
                    "and talk to them.",
            "Be careful.",
            "Oan I get help\n"+
                    "with the Sky Mall magazine?",
            "I'd like to order the talking\n"+
                    "inflatable nose and ear hair trimmer.",
            "Oaptain, I'm in a real situation.",
            "- What'd you say, Hal?\n"+
                    "- Nothing.",
            "Bee!",
            "Don't freak out! My entire species...",
            "What are you doing?",
            "- Wait a minute! I'm an attorney!\n"+
                    "- Who's an attorney?",
            "Don't move.",
            "Oh, Barry.",
            "Good afternoon, passengers.\n"+
                    "This is your captain.",
            "Would a Miss Vanessa Bloome in 24B\n"+
                    "please report to the cockpit?",
            "And please hurry!",
            "What happened here?",
            "There was a DustBuster, a toupee, a life raft exploded.",
            "One's bald, one's in a boat, they're both unconscious!",
            "- Is that another bee joke?\n"+
                    "- No!",
            "No one's flying the plane!",
            "This is JFK control tower, Flight 356.\n"+
                    "What's your status?",
            "This is Vanessa Bloome.\n"+
                    "I'm a florist from New York.",
            "Where's the pilot?",
            "He's unconscious, and so is the copilot.",
            "Not good. Does anyone onboard\n"+
                    "have flight experience?",
            "As a matter of fact, there is.",
            "- Who's that?\n"+
                    "- Barry Benson.",
            "From the honey trial?! Oh, great.",
            "Vanessa, this is nothing more\n"+
                    "than a big metal bee.",
            "It's got giant wings, huge engines.",
            "I can't fly a plane.",
            "- Why not? Isn't John Travolta a pilot?\n"+
                    "- Yes.",
            "How hard could it be?",
            "Wait, Barry!\n"+
                    "We're headed into some lightning.",
            "This is Bob Bumble. We have some\n"+
                    "late-breaking news from JFK Airport,",
            "where a suspenseful scene\n"+
                    "is developing.",
            "Barry Benson, fresh from his legal victory...",
            "That's Barry!",
            "...is attempting to land a plane, loaded with people, flowers",
            "and an incapacitated flight crew.",
            "Flowers?!",
            "We have a storm in the area\n"+
                    "and two individuals at the controls",
            "with absolutely no flight experience.",
            "Just a minute.\n"+
                    "There's a bee on that plane.",
            "I'm quite familiar with Mr. Benson\n"+
                    "and his no-account compadres.",
            "They've done enough damage.",
            "But isn't he your only hope?",
            "Technically, a bee\n"+
                    "shouldn't be able to fly at all.",
            "Their wings are too small...",
            "Haven't we heard this a million times?",
            "\"The surface area of the wings\n"+
                    "and body mass make no sense.\"",
            "- Get this on the air!\n"+
                    "- Got it.",
            "- Stand by.\n"+
                    "- We're going live.",
            "The way we work may be a mystery to you.",
            "Making honey takes a lot of bees\n"+
                    "doing a lot of small jobs.",
            "But let me tell you about a small job.",
            "If you do it well, it makes a big difference.",
            "More than we realized.\n"+
                    "To us, to everyone.",
            "That's why I want to get bees\n"+
                    "back to working together.",
            "That's the bee way!\n"+
                    "We're not made of Jell-O.",
            "We get behind a fellow.",
            "- Black and yellow!\n"+
                    "- Hello!",
            "Left, right, down, hover.",
            "- Hover?\n"+
                    "- Forget hover.",
            "This isn't so hard.\n"+
                    "Beep-beep! Beep-beep!",
            "Barry, what happened?!",
            "Wait, I think we were\n"+
                    "on autopilot the whole time.",
            "- That may have been helping me.\n"+
                    "- And now we're not!",
            "So it turns out I cannot fly a plane.",
            "All of you, let's get\n"+
                    "behind this fellow! Move it out!",
            "Move out!",
            "Our only chance is if I do what I'd do, you copy me with the wings of the plane!",
            "Don't have to yell.",
            "I'm not yelling!\n"+
                    "We're in a lot of trouble.",
            "It's very hard to concentrate\n"+
                    "with that panicky tone in your voice!",
            "It's not a tone. I'm panicking!",
            "I can't do this!",
            "Vanessa, pull yourself together.\n"+
                    "You have to snap out of it!",
            "You snap out of it.",
            "You snap out of it.",
            "- You snap out of it!\n"+
                    "- You snap out of it!",
            "- You snap out of it!\n"+
                    "- You snap out of it!",
            "- You snap out of it!\n"+
                    "- You snap out of it!",
            "- Hold it!\n"+
                    "- Why? Oome on, it's my turn.",
            "How is the plane flying?",
            "I don't know.",
            "Hello?",
            "Benson, got any flowers\n"+
                    "for a happy occasion in there?",
            "The Pollen Jocks!",
            "They do get behind a fellow.",
            "- Black and yellow.\n"+
                    "- Hello.",
            "All right, let's drop this tin can\n"+
                    "on the blacktop.",
            "Where? I can't see anything. Oan you?",
            "No, nothing. It's all cloudy.",
            "Oome on. You got to think bee, Barry.",
            "- Thinking bee.\n"+
                    "- Thinking bee.",
            "Thinking bee!\n"+
                    "Thinking bee! Thinking bee!",
            "Wait a minute.\n"+
                    "I think I'm feeling something.",
            "- What?\n"+
                    "- I don't know. It's strong, pulling me.",
            "Like a 27-million-year-old instinct.",
            "Bring the nose down.",
            "Thinking bee!\n"+
                    "Thinking bee! Thinking bee!",
            "- What in the world is on the tarmac?\n"+
                    "- Get some lights on that!",
            "Thinking bee!\n"+
                    "Thinking bee! Thinking bee!",
            "- Vanessa, aim for the flower.\n"+
                    "- OK.",
            "Out the engines. We're going in\n"+
                    "on bee power. Ready, boys?",
            "Affirmative!",
            "Good. Good. Easy, now. That's it.",
            "Land on that flower!",
            "Ready? Full reverse!",
            "Spin it around!",
            "- Not that flower! The other one!\n"+
                    "- Which one?",
            "- That flower.\n"+
                    "- I'm aiming at the flower!",
            "That's a fat guy in a flowered shirt.\n"+
                    "I mean the giant pulsating flower",
            "made of millions of bees!",
            "Pull forward. Nose down. Tail up.",
            "Rotate around it.",
            "- This is insane, Barry!\n"+
                    "- This's the only way I know how to fly.",
            "Am I koo-koo-kachoo, or is this plane\n"+
                    "flying in an insect-like pattern?",
            "Get your nose in there. Don't be afraid.\n"+
                    "Smell it. Full reverse!",
            "Just drop it. Be a part of it.",
            "Aim for the center!",
            "Now drop it in! Drop it in, woman!",
            "Oome on, already.",
            "Barry, we did it!\n"+
                    "You taught me how to fly!",
            "- Yes. No high-five!\n"+
                    "- Right.",
            "Barry, it worked!\n"+
                    "Did you see the giant flower?",
            "What giant flower? Where? Of course\n"+
                    "I saw the flower! That was genius!",
            "- Thank you.\n"+
                    "- But we're not done yet.",
            "Listen, everyone!",
            "This runway is covered\n"+
                    "with the last pollen",
            "from the last flowers\n"+
                    "available anywhere on Earth.",
            "That means this is our last chance.",
            "We're the only ones who make honey, pollinate flowers and dress like this.",
            "If we're gonna survive as a species, this is our moment! What do you say?",
            "Are we going to be bees, orjust\n"+
                    "Museum of Natural History keychains?",
            "We're bees!",
            "Keychain!",
            "Then follow me! Except Keychain.",
            "Hold on, Barry. Here.",
            "You've earned this.",
            "Yeah!",
            "I'm a Pollen Jock! And it's a perfect\n"+
                    "fit. All I gotta do are the sleeves.",
            "Oh, yeah.",
            "That's our Barry.",
            "Mom! The bees are back!",
            "If anybody needs\n"+
                    "to make a call, now's the time.",
            "I got a feeling we'll be\n"+
                    "working late tonight!",
            "Here's your change. Have a great\n"+
                    "afternoon! Oan I help who's next?",
            "Would you like some honey with that?\n"+
                    "It is bee-approved. Don't forget these.",
            "Milk, cream, cheese, it's all me.\n"+
                    "And I don't see a nickel!",
            "Sometimes I just feel\n"+
                    "like a piece of meat!",
            "I had no idea.",
            "Barry, I'm sorry.\n"+
                    "Have you got a moment?",
            "Would you excuse me?\n"+
                    "My mosquito associate will help you.",
            "Sorry I'm late.",
            "He's a lawyer too?",
            "I was already a blood-sucking parasite.\n"+
                    "All I needed was a briefcase.",
            "Have a great afternoon!",
            "Barry, I just got this huge tulip order, and I can't get them anywhere.",
            "No problem, Vannie.\n"+
                    "Just leave it to me.",
            "You're a lifesaver, Barry.\n"+
                    "Oan I help who's next?",
            "All right, scramble, jocks!\n"+
                    "It's time to fly.",
            "Thank you, Barry!",
            "That bee is living my life!",
            "Let it go, Kenny.",
            "- When will this nightmare end?!\n"+
                    "- Let it all go.",
            "- Beautiful day to fly.\n"+
                    "- Sure is.",
            "Between you and me,I was dying to get out of that office.",
            "You have got\n"+
                    "to start thinking bee, my friend.",
            "- Thinking bee!\n"+
                    "- Me?",
            "Hold it. Let's just stop\n"+
                    "for a second. Hold it.",
            "I'm sorry. I'm sorry, everyone.\n"+
                    "Oan we stop here?",
            "I'm not making a major life decision\n"+
                    "during a production number!",
            "All right. Take ten, everybody.\n"+
                    "Wrap it up, guys.",
            "I had virtually no rehearsal for that."
    };
    public BeeMovie() {
        super(0, true);
    }

    @Override
    protected void setup() {
        telemetry.enableLogger();
        HashMap<String, LogicState> stateList = new HashMap<>();
        for (int i = 0; i < script.length; i++) {
            final String line = script[i];
            final int thisState = i;
            stateList.put(String.valueOf(i), new LogicState(stateMachine) {
                long lastTime = 0;
                @Override
                public void update(ReadData data) {
                    if(lastTime == 0){
                        lastTime = System.currentTimeMillis();
                        telemetry.setHeader("script", line);
                    }
                    else if(System.currentTimeMillis()-lastTime>=1000){
                        deactivateThis();
                        if(thisState==script.length-1){
                            stop();
                        }
                        else {
                            stateMachine.activateLogic(String.valueOf(thisState + 1));
                        }
                    }
                }
            });
        }
        stateList.put("-1", new LogicState(stateMachine) {

            @Override
            public void update(ReadData data) {
                if(isStarted()){
                    stateMachine.activateLogic("0");
                    deactivateThis();
                }
            }
        });
        stateMachine.appendLogicStates(stateList);
        stateMachine.activateLogic("-1");
    }
}