
from llama_cpp import Llama

llm = Llama.from_pretrained(
	repo_id="Qwen/Qwen2.5-0.5B-Instruct-GGUF",
	filename="qwen2.5-0.5b-instruct-q5_0.gguf",
	cache_dir="./"
)

llm.create_chat_completion(
	messages = [
		{
			"role": "user",
			"content": "What is the capital of France?"
		}
	]
)


# # Load model directly
# from transformers import AutoTokenizer, AutoModelForCausalLM

# # 저장하고 싶은 경로 설정
# save_path = "./"

# tokenizer = AutoTokenizer.from_pretrained("Qwen/Qwen2.5-0.5B-Instruct", cache_dir=save_path)
# model = AutoModelForCausalLM.from_pretrained("Qwen/Qwen2.5-0.5B-Instruct", cache_dir=save_path)
# messages = [
#     {"role": "user", "content": "Who are you?"},
# ]
# inputs = tokenizer.apply_chat_template(
# 	messages,
# 	add_generation_prompt=True,
# 	tokenize=True,
# 	return_dict=True,
# 	return_tensors="pt",
# ).to(model.device)

# outputs = model.generate(**inputs, max_new_tokens=40)
# print(tokenizer.decode(outputs[0][inputs["input_ids"].shape[-1]:]))