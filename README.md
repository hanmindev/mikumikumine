# MikuMine
MikuMine is a program that converts .VMD files from mikumikudance to minecraft functions so you can see the dances in-game.
Example:
https://youtu.be/gpqPGgSCI2E

Minecraft world file:
https://www.mediafire.com/file/1tte0k23mt9qzgx/mikumikudance.zip/file

# Operation
Operation is quite simple. First, download the minecraft world file and save it in your saves folder. Run main.py, then when prompted for a file, select your .vmd animation file. On the next prompt, navigate to your minecraft saves folder, enter the world file, datapacks/mmd/data/mmd/functions/FOLDERNAME with foldername being whatever you want.

After the program halts, go into the parent folder of FOLDERNAME. There, open dance.mcfunction, and change FOLDERNAME to whatever you named your folder.

To add additional animations, you must copy paste a few lines in both dance.mcfunction and masterloop.mcfunction. There are instructions in the comments.

I didn't include any dances in the world file because I don't own the animations, but in theory any .VMD dance file should work as long as it:  
1. was created using the Multi-Model Edition
2. has the following bones: center, groove, upper body, upper body2, shoulder L, arm L, elbow L, wrist L, shoulder R, arm R, elbow R, wrist R, neck, waist, leg L, knee L, ankle L, toe L, leg R, knee R, ankle R, toe R, main, mother, head. (They have to be named in Japanese. You can have additional bones, they'll just get ignored.)
3. does not use interpolation. Perhaps I'll add this in the future when I have more time but as of right now, I really don't want to learn quaternion interpolation.

I've only tried importing animations from idolmaster million live theater days (using https://github.com/OpenMLTD/MLTDTools) so it might not work for different .vmd files. Open an issue and send your .VMD file and I'll look into it.
